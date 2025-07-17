/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

// FreeRTOS (Pure) Includes
extern "C" {
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
}

extern "C" {
#include "board.h"
#include "common.h"
#include "main.h"
}

#include <cctype>  // isprint()

#include "RxStatemachine.hpp"
#include "TxStatemachine.hpp"
#include "chirptables.hpp"
#include "ecc.hpp"
#include "fifo.hpp"
#include "fontus.hpp"
#include "fontusmodem.hpp"
#include "hoppattern.hpp"
#include "phasor.hpp"
#include "popoto_types.hpp"
#include "testvect.h"
#include "trellis.hpp"
#include <iomanip>    // for std::setw/std::left - needed for both STM32 and emulator
#ifdef STM32L496xx
#include "SEGGER_RTT.h"
extern volatile uint32_t xrun_count;
/* SEGGER_RTT_MEMCPY_USE_BYTELOOP already defined in SEGGER_RTT_Conf.h */
#ifdef USE_RTT

// Custom stream buffer that writes to RTT
class RTTStreamBuffer : public std::streambuf {
 protected:
  int overflow(int c) override {
    if (c != EOF) {
      char z = static_cast<char>(c);
      SEGGER_RTT_Write(0, &z, 1);  // Write a single character to RTT channel 0
    }
    return c;
  }

  std::streamsize xsputn(const char *s, std::streamsize n) override {
    SEGGER_RTT_Write(0, s, static_cast<unsigned>(n));  // Write to RTT
    return n;
  }
};

// Redirect `std::cout` to RTT
void redirectStdoutToRTT() {
  static RTTStreamBuffer rttBuf;
  static std::ostream rttOut(&rttBuf);
  std::cout.rdbuf(rttOut.rdbuf());
}

#else  // USE_RTT is NOT defined -> Include USB code

// For huart3 and HAL_UART functions

class UARTStreamBuffer : public std::streambuf {
 protected:
  /**
   * @brief Called when the put area is full or a special character triggers
   * flush. Here, we transmit every single character directly via UART.
   */
  int overflow(int c) override {
    if (c != EOF) {
      char chr = static_cast<char>(c);

      // Replace '\n' with "\r\n" for typical serial terminal formatting
      if (chr == '\n') {
        const char newline[2] = {'\r', '\n'};
        HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t *>(newline),
                          2, HAL_MAX_DELAY);
      } else {
        HAL_UART_Transmit(&huart3, reinterpret_cast<const uint8_t *>(&chr), 1,
                          HAL_MAX_DELAY);
      }
    }
    return c;
  }

  /**
   * @brief Writes a sequence of characters. We just call `overflow()` on each
   * char.
   */
  std::streamsize xsputn(const char *s, std::streamsize n) override {
    for (std::streamsize i = 0; i < n; ++i) {
      overflow(s[i]);
    }
    return n;
  }
};

/**
 * @brief Redirect std::cout to UART, transmitting one character at a time.
 */
void redirectStdoutToUART() {
  static UARTStreamBuffer uartStreamBuf;
  static std::ostream uartStream(&uartStreamBuf);
  std::cout.rdbuf(uartStream.rdbuf());
}

#endif  // USE_RTT

// ─── Hardware staging buffer ─────────────────────────────────────────────────

#endif  // STM32L496xx

#ifndef STM32L496xx
const char *g_inputFilePath =
    "../fontusmodem/vectors/testvect.packed";  // default
#endif

uint32_t inBuffer[CHUNK_BYTES];

/* Definitions ---------------------------------------------------------------*/
#define FAM_TASK_BLOCKING_MS 1000
#define MAX_FFT_SIZE (8192 * 4)
volatile bool g_enableRx = false;
volatile bool g_enableTx = false;
volatile uint32_t dmaBufferHalf = 0;  // 0 = first half, 1 = second half

/* Fontus configuration ------------------------------------------------------*/
FontusConfig cfg = {hop_values,
                    144,
                    64000,
                    25000,
                    0.0625f,
                    4000,
                    4,
                    0.2f,
                    0.05f,
                    0.1f,
                    1500,
                    0.00625,
                    320,
                    11,
                    wupchirp16table,
                    WUPCHIRP16LEN,
                    dop_upchirp16table,
                    DOP_UPCHIRP16LEN,
                    dop_dnchirp16table,
                    DOP_DNCHIRP16LEN,
                    12.0f};

Fontus modem(&cfg);

/* Alignment requirement for DSP operations ----------------------------------*/
int8_t tmpbuf[MAX_FFT_SIZE * 2] __attribute__((aligned(8)));

/* Global Variables (FreeRTOS Task Handles, Queue) ---------------------------*/
TaskHandle_t mainTaskHandle = nullptr;
TaskHandle_t FAM_TaskHandle = nullptr;
TaskHandle_t consoleTaskHandle = nullptr;
TaskHandle_t  txTaskHandle = nullptr;
QueueHandle_t xFAMQueue = nullptr;
SemaphoreHandle_t filledSem = nullptr;
SemaphoreHandle_t txSemaphore = nullptr;  // For TX task wakeup
/* DSP Instances -------------------------------------------------------------*/
arm_rfft_instance_q15 S15;
arm_rfft_fast_instance_f32 S32;

/* Heap usage tracker --------------------------------------------------------*/
size_t totalHeapUsage = 0;

/* Overload global new/delete to track heap usage ----------------------------*/
void *operator new(size_t size) {
  totalHeapUsage += size;
  void *ptr = calloc(size, sizeof(char));
  if (!ptr) {
    // Stay in an infinite loop if allocation fails
    while (true) {
      // Error handling
    }
  }
  return ptr;
}

void operator delete(void *ptr) noexcept { free(ptr); }
void operator delete(void *ptr, std::size_t) noexcept { free(ptr); }

/* Forward Declarations ------------------------------------------------------*/
static void StartMainTask(void *pvParameters);
static void StartFAMTask(void *pvParameters);
static void StartConsoleTask(void *pvParameters);
static void StartTXTask(void *pvParameters);
/**
 * @brief  The application entry point (Pure FreeRTOS version).
 */
int main(int argc, char *argv[]) {
#ifdef STM32L496xx
  (void)argc;
  (void)argv;
  SEGGER_RTT_Init();
#ifdef USE_RTT

  redirectStdoutToRTT();
#else
  redirectStdoutToUART();
#endif
#endif
#ifndef STM32L496xx
  if (argc > 1) {
    g_inputFilePath = argv[1];
  }
#endif
  // Hardware / Board initialization
  Board_Init();
  osCOMP1_Start();  // Temporarily disable to test DAC output 
  osCOMP2_Start();

  filledSem = xSemaphoreCreateBinary();
  if (!filledSem) {
    std::cout << "Semaphore creation failed!\n";
    Error_Handler();
  }
  // txSemaphore no longer needed - using DAC DMA with timer triggering
  // Create queue
  xFAMQueue =
      xQueueCreate(1, sizeof(uint32_t));  // 1 item, each of size uint32_t
  if (xFAMQueue == NULL) {
    std::cout << "Queue creation failed!\n";
    Error_Handler();
  }

  // Create tasks (pure FreeRTOS)
  BaseType_t result;

  result = xTaskCreate(StartMainTask,
                       "MainTask",            // Task name
                       1024,                  // Stack size in words
                       nullptr,               // Parameters
                       tskIDLE_PRIORITY + 2,  // Priority
                       &mainTaskHandle);
  if (result != pdPASS) {
    std::cout << "Failed to create MainTask!\n";
    Error_Handler();
  }

  result = xTaskCreate(StartFAMTask, "FAMTask", 1024, nullptr,
                       tskIDLE_PRIORITY + 3, &FAM_TaskHandle);
  if (result != pdPASS) {
    std::cout << "Failed to create FAMTask!\n";
    Error_Handler();
  }
  result = xTaskCreate(StartConsoleTask,  // NEW
                       "Console",
                       512,  // stack words (plenty)
                       nullptr,
                       tskIDLE_PRIORITY + 4,  // higher prio than FAM/Main
                       &consoleTaskHandle);
      
  if (result != pdPASS) {
    std::cout << "Failed to create ConsoleTask!\n";
    Error_Handler();
  }
 result = xTaskCreate(StartTXTask,
                       "TXTask",
                       2048,
                       nullptr,
                       tskIDLE_PRIORITY + 3,
                       &txTaskHandle);
  if (result != pdPASS) {
    std::cout << "Failed to create TXTask!\n";
    Error_Handler();
  }
#ifdef STM32L496xx
  configureTimerForRunTimeStats();
#endif

  vTaskStartScheduler();

  // If all is well, we will never reach here.
  // If we do, it indicates insufficient FreeRTOS heap or another error.
  while (1) {
    // Infinite loop if scheduler fails
  }
}

/**
 * @brief Main task implementation (pure FreeRTOS).
 */
static void StartMainTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {

    modem.rxsm->receive();  // Call only when enabled
  }
}

/**
 * @brief FAM task implementation (pure FreeRTOS).
 */
static void StartFAMTask(void *pvParameters) {
  (void)pvParameters;  // Avoid unused parameter warning

  // Example hardware init call

  for (;;) {
    // timeout / EOF
      if (!g_enableRx) {
      vTaskDelay(pdMS_TO_TICKS(FAM_TASK_BLOCKING_MS));
      continue;  // Skip processing if RX is not enabled
    }
    if (modem.pcminpacked32->spaceavailable() >= CHUNK_BYTES) {
      osReceive(inBuffer, CHUNK_BYTES);

      modem.pcminpacked32->write(inBuffer,
                                 CHUNK_SAMPLES);  // 2 bytes per sample

      }
  }
}
/* ─── Console Command Infrastructure ─────────────────────────────────────── */

using CmdFn = void (*)(const char *args);  // optional arg string after cmd

struct ConsoleCommand {
  const char *name;
  CmdFn handler;
  const char *description;  // ← Add this field
};

/* — Example command handlers — */
static void cmd_fifo(const char *) {
  uint32_t freeB = modem.pcminpacked32->spaceavailable();
  std::cout << "Packed FIFO free: " << freeB << " bytes\n";
}
static void cmd_startrx(const char *) {
  g_enableRx = true;
  std::cout << "Started RX\n";
}
static void cmd_stoprx(const char *) {
  g_enableRx = false;
  std::cout << "Stopped RX\n";
}

/* arm the modem TX state-machine and enable the power amp */
/* ----  Console “starttx” command (uses DatEnc::dataencode)  ---- */
// Console “starttx” handler
static void cmd_starttx(const char * /*args*/)
{
    if (g_enableTx) {
        std::cout << "TX already running\r\n";
        return;
    }

    g_enableTx = true;   // TX task will start spinning immediately
    BoostEnable(1);   // enable 5V boost for DAC
    GateEnable(1);    // power-amp output (if you use it)
    Buck10VEnable(1); // enable 10V buck for PA
    // Start TIM1 for 64kHz test GPIO
  

    std::cout << "TX started with Fontus modem\r\n";
}




/* stop TX immediately */
static void cmd_stoptx(const char * /*args*/)
{
    if (!g_enableTx) {
        std::cout << "TX not running\r\n";
        return;
    }

    g_enableTx = false;  // TX task will fall back to 10-ms sleeps
    GateEnable(0);   // mute PA
    
    // Stop TIM1 test GPIO
    osStopTIM();

    std::cout << "TX stopped\r\n";
}
static void cmd_uptime(const char *) {
  TickType_t ticks = xTaskGetTickCount();
  uint32_t seconds = ticks / configTICK_RATE_HZ;

  uint32_t days = seconds / 86400;
  uint32_t hours = (seconds % 86400) / 3600;
  uint32_t minutes = (seconds % 3600) / 60;
  uint32_t secs = seconds % 60;

  std::cout << "Uptime: ";
  if (days) std::cout << days << "d ";
  if (hours) std::cout << hours << "h ";
  if (minutes) std::cout << minutes << "m ";
  std::cout << secs << "s\n";
}
static void cmd_clk(const char *) {
#ifdef STM32L496xx
  uint32_t freqHz = HAL_RCC_GetSysClockFreq();
  std::cout << "System clock: " << freqHz / 1'000'000 << " MHz\n";
#else
  std::cout << "System clock info not available on this platform.\n";
#endif
}
/* — CPU run-time stats command — */
static void cmd_cpu(const char *)
{
    // 1) Print a banner
    std::cout 
      << "\r\n"
      << "==== FreeRTOS CPU Run-Time Stats ====\r\n"
      << "Task Name          Abs Time      % Time\r\n"
      << "----------------------------------------\r\n";

    // 2) Dump the stats (vTaskGetRunTimeStats writes rows like: “IDLE            12345      12%”)
    static char statsBuf[512];
    vTaskGetRunTimeStats(statsBuf);
    std::cout << statsBuf;

    // 3) Optionally a footer
    std::cout 
      << "========================================\r\n"
      << "\r\n";
}

static void cmd_stack(const char *)
{
    // 1) Allocate space for all tasks
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    TaskStatus_t *statusArray = (TaskStatus_t*) pvPortMalloc(taskCount * sizeof(TaskStatus_t));
    if (!statusArray) {
        std::cout << "Out of memory!\r\n";
        return;
    }

    // 2) Populate statusArray
    UBaseType_t retrieved = uxTaskGetSystemState(statusArray, taskCount, nullptr);

    // 3) Print our own header
    std::cout
      << "Name            State  Prio  StackW  StackB    Num\r\n"
      << "****************************************************\r\n";

    // 4) For each task, compute bytes = words * sizeof(StackType_t)
    for (UBaseType_t i = 0; i < retrieved; ++i) {
        auto &s = statusArray[i];

        // map FreeRTOS eTaskState → single char
        char st = '?';
        switch (s.eCurrentState) {
          case eRunning:   st = 'R'; break;
          case eReady:     st = 'R'; break;
          case eBlocked:   st = 'B'; break;
          case eSuspended: st = 'S'; break;
          case eDeleted:   st = 'D'; break;
          default:         st = '?'; break;
        }

        UBaseType_t words = s.usStackHighWaterMark;               // never-used words
        uint32_t    bytes = words * sizeof(StackType_t);          // bytes free
        // Now print: Name, State, Prio, StackW, StackB, Task Number
        std::cout
          << std::setw(15) << std::left << s.pcTaskName << " "
          << st << "      "
          << std::setw(3)  << s.uxCurrentPriority        << "   "
          << std::setw(6)  << words                      << "   "
          << std::setw(8)  << bytes                      << "   "
          << s.xTaskNumber
          << "\r\n";
    }

    // 5) Clean up
    vPortFree(statusArray);
} 


// —––– Heap usage command —–––––––––––––––––––––––––––––––––––––––––––––––––

static void cmd_heap(const char *)
{
#ifdef STM32L496xx
    // 1) Gather stats
    const size_t totalHeap  = configTOTAL_HEAP_SIZE;
    const size_t freeNow    = xPortGetFreeHeapSize();
    const size_t minEver    = xPortGetMinimumEverFreeHeapSize();

    const size_t usedNow    = totalHeap - freeNow;
    const size_t peakUsed   = totalHeap - minEver;

    const int    pctUsed    = (int)((usedNow * 100) / totalHeap);
    const int    pctFree    = 100 - pctUsed;

    // 2) Print banner & table
    std::cout 
      << "\r\n==== Heap Usage ====\r\n"
      << std::setw(18) << std::left << "Total Heap:"     << totalHeap   << " bytes\r\n"
      << std::setw(18) << std::left << "Used Now:"       << usedNow     << " bytes (" << pctUsed << "%)\r\n"
      << std::setw(18) << std::left << "Free Now:"       << freeNow     << " bytes (" << pctFree << "%)\r\n"
      << std::setw(18) << std::left << "Peak Used:"      << peakUsed    << " bytes (" << (int)((peakUsed*100)/totalHeap) << "%)\r\n"
      << std::setw(18) << std::left << "Min Ever Free:"  << minEver     << " bytes\r\n"
      << "====================\r\n\r\n";
#else
    std::cout << "Heap stats not available in emulator build\r\n";
#endif
}

static void cmd_gpio(const char *args)
{
    if (!args || strlen(args) < 2) {
        std::cout << "Usage: gpio <port><pin> [0|1]\r\n";
        std::cout << "Examples: gpio a7, gpio b3 1, gpio c12 0\r\n";
        return;
    }

    // Parse port (A, B, C, etc.)
    char port = toupper(args[0]);
    if (port < 'A' || port > 'H') {
        std::cout << "Invalid port. Use A-H\r\n";
        return;
    }

    // Parse pin number
    int pin = atoi(&args[1]);
    if (pin < 0 || pin > 15) {
        std::cout << "Invalid pin. Use 0-15\r\n";
        return;
    }

    // Get GPIO port
    GPIO_TypeDef *gpio_port;
    switch (port) {
        case 'A': gpio_port = GPIOA; break;
        case 'B': gpio_port = GPIOB; break;
        case 'C': gpio_port = GPIOC; break;
        case 'D': gpio_port = GPIOD; break;
        case 'E': gpio_port = GPIOE; break;
        case 'F': gpio_port = GPIOF; break;
        case 'G': gpio_port = GPIOG; break;
        case 'H': gpio_port = GPIOH; break;
        default:
            std::cout << "Unsupported port\r\n";
            return;
    }

    uint16_t gpio_pin = 1 << pin;

    // Find value argument (skip port+pin)
    const char *value_str = args;
    while (*value_str && !isspace(*value_str)) value_str++;
    while (*value_str && isspace(*value_str)) value_str++;

    if (*value_str == '\0') {
        // No argument - toggle the pin
        GPIO_PinState currentState = osGPIORead(gpio_port, gpio_pin);
        GPIO_PinState newState = (currentState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        osGPIOWrite(gpio_port, gpio_pin, newState);
        std::cout << "GPIO " << port << pin << " toggled to " << (newState ? "HIGH" : "LOW") << "\r\n";
    } else {
        // Parse argument: 0 = LOW, 1 = HIGH
        int value = atoi(value_str);
        GPIO_PinState state = (value != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        osGPIOWrite(gpio_port, gpio_pin, state);
        std::cout << "GPIO " << port << pin << " set to " << (state ? "HIGH" : "LOW") << "\r\n";
    }
}
static void cmd_help(const char *);
/* — Command table (NULL-terminated) — */
static const ConsoleCommand cmdTable[] = {
    {"fifo", cmd_fifo, "Show free space in packed FIFO"},
    {"startrx", cmd_startrx, "Enable RX mode"},
    {"stoprx", cmd_stoprx, "Disable RX mode"},
    {"starttx", cmd_starttx, "Enable TX state machine"},
    {"stoptx",  cmd_stoptx,  "Disable TX state machine"},
    {"uptime", cmd_uptime, "Show system uptime"},
    {"clk", cmd_clk, "Display system clock frequency"},
    {"cpu",     cmd_cpu,     "Show FreeRTOS CPU usage per task"},
    {"stack",   cmd_stack,   "Show FreeRTOS task stack usage"},
    {"heap",    cmd_heap,    "Show FreeRTOS heap usage"},
    {"gpio",    cmd_gpio,    "Control GPIO pins (gpio <port><pin> [0|1])"},
    {"help", cmd_help, "Show available commands"},
    {nullptr, nullptr, nullptr}};

// Now define cmd_help AFTER cmdTable
static void cmd_help(const char *) {
  std::cout << "Available commands:\n";
  for (const ConsoleCommand *c = cmdTable; c->name; ++c) {
    std::cout << "  " << c->name;
    if (c->description) {
      std::cout << " - " << c->description;
    }
    std::cout << "\n";
  }
}

// ────────────────────────────────────────────────────────────────
// Console task with non-blocking ANSI parser and 10-line history
// ────────────────────────────────────────────────────────────────
static void StartConsoleTask(void *) {
  constexpr const char *PROMPT = "nfwf_board-> ";
  constexpr size_t LINE_LEN = 64;
  constexpr size_t HIST_DEPTH = 10;

  // Make std::cout unbuffered
  std::cout << std::unitbuf;

  // Circular history buffer
  static char history[HIST_DEPTH][LINE_LEN] = {};
  int hist_head = 0;  // next slot to write
  int hist_pos = 0;   // current browse position

  auto store_history = [&](const char *cmd) {
    if (!*cmd) return;
    strncpy(history[hist_head], cmd, LINE_LEN);
    history[hist_head][LINE_LEN - 1] = '\0';
    hist_head = (hist_head + 1) % HIST_DEPTH;
    hist_pos = hist_head;
  };

  auto redraw = [&](const char *buf) {
    // Clear line, return to col 0, print prompt + buffer
    std::cout << "\r\x1b[2K" << PROMPT << buf;
  };

  enum EscState { NORMAL, ESC_SEEN, CSI_SEEN };
  EscState esc = NORMAL;

  char lineBuf[LINE_LEN] = {};
  size_t idx = 0;

  // Initial prompt
  std::cout << "\r\x1b[2K" << PROMPT;

  auto dispatch = [&](char *line) {
    store_history(line);
    char *argPtr = strchr(line, ' ');
    if (argPtr) *argPtr++ = '\0';

    for (const ConsoleCommand *c = cmdTable; c->name; ++c) {
      if (strcmp(line, c->name) == 0) {
        c->handler(argPtr);
        return;
      }
    }
    std::cout << "?? unknown cmd: " << line << "\r\n";
  };

  while (true) {
    int ch = osConsole_ReadByte();
    if (ch < 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Handle escape sequences
    if (esc != NORMAL) {
      if (esc == ESC_SEEN) {
        esc = (ch == '[') ? CSI_SEEN : NORMAL;
        continue;
      }
      if (esc == CSI_SEEN) {
        if (ch >= '@' && ch <= '~') {
          if (ch == 'A' || ch == 'B') {
            int next = (ch == 'A') ? (hist_pos - 1 + HIST_DEPTH) % HIST_DEPTH
                                   : (hist_pos + 1) % HIST_DEPTH;
            if (next != hist_head && history[next][0]) {
              strcpy(lineBuf, history[next]);
              idx = strlen(lineBuf);
              hist_pos = next;
            } else if (next == hist_head) {
              idx = 0;
              lineBuf[0] = '\0';
              hist_pos = hist_head;
            }
            redraw(lineBuf);
          }
          esc = NORMAL;
        }
        continue;
      }
    }
    if (ch == 0x1B) {
      esc = ESC_SEEN;
      continue;
    }

    // Line editing
    if (ch == '\r' || ch == '\n') {
      std::cout << "\r\n";
      if (idx) {
        lineBuf[idx] = '\0';

        // Trim leading spaces
        char *trimmed = lineBuf;
        while (*trimmed == ' ') ++trimmed;

        // Trim trailing spaces
        size_t len = strlen(trimmed);
        while (len > 0 && trimmed[len - 1] == ' ') trimmed[--len] = '\0';

        if (*trimmed) {
          dispatch(trimmed);
        } else {
          std::cout << "\r\n";  // Just newline for empty input
        }

        idx = 0;
        lineBuf[0] = '\0';  // Clear buffer!
      }

      // Clear line + prompt
      std::cout << "\r\x1b[2K" << PROMPT;
    } else if (ch == 0x7F || ch == 0x08) {
      if (idx) {
        --idx;
        std::cout << "\b \b";
      }
    } else if (isprint(ch) && idx < LINE_LEN - 1) {
      lineBuf[idx++] = static_cast<char>(ch);
      std::cout << static_cast<char>(ch);
    }
    // ignore all other bytes
  }
}
static void StartTXTask(void * /*pvParameters*/)
{
    int16_t *txBuffer = nullptr;
    int32_t bufferSize = 0;
    bool transmitting = false;
    
    while (true) {
        if (!g_enableTx) {
            if (transmitting) {
                fontus_tx_stop();
                transmitting = false;
                bufferSize = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (!transmitting) {
            /* Start new transmission */
            std::cout << "TX: Starting transmission...\r\n";
            
            uint8_t mybytes[16] = {0x32, 0x31, 0x10, 0x55, 0x77, 0x33, 0xEE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int16_t nbytes = modem.txsm->pktenc->dataencode(mybytes, 7);
            modem.txsm->send(mybytes, nbytes);
            
            /* Allocate buffer for one chunk of samples */
            bufferSize = modem.chiplen;
            txBuffer = new int16_t[bufferSize];
            
            transmitting = true;
            std::cout << "TX: Transmission started, chiplen=" << bufferSize << "\r\n";
        }
        
        if (transmitting) {
            /* Get full waveform from modem once */
            int32_t result = modem.txsm->run(txBuffer);
            
            if (result == 0) {
                /* Modem has data to transmit - send entire buffer to DMA */
                osTransmit(txBuffer, bufferSize);
                fontus_tx_start();  // Start DMA once
                
                
                /* Wait for transmission to complete (time-based) */
                uint32_t transmissionTimeMs = (bufferSize * 1000) / cfg.fs + 10;  // Add 10ms margin
                vTaskDelay(pdMS_TO_TICKS(transmissionTimeMs));
                
                /* Continue to next chunk or finish */
                continue;
            } else {
                /* Transmission complete - stop TX */
                std::cout << "TX: Transmission complete - stopping TX\r\n";
                fontus_tx_stop();
                transmitting = false;
                delete[] txBuffer;
                txBuffer = nullptr;
                
                /* Disable TX mode and power amp */
                g_enableTx = false;
                GateEnable(false);
                
                /* Stop TIM1 */
                osStopTIM();
                
                std::cout << "TX stopped\r\n";
                continue;
            }
        }
    }
}

/**
 * @brief Error Handler (same as before).
 */
void Error_Handler(void) {
  while (1) {
    // Infinite loop in case of error
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the source file and line where assert_param error occurred.
 */
void assert_failed(uint8_t *file, uint32_t line) {
  std::cout << "Wrong parameters value: file " << file << " on line " << line
            << "\n";
}
#endif
