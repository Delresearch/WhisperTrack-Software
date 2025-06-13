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
#ifdef STM32L496xx
#include "SEGGER_RTT.h"

extern volatile uint32_t xrun_count;
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP 1
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
QueueHandle_t xFAMQueue = nullptr;
SemaphoreHandle_t filledSem = nullptr;
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

/* Forward Declarations ------------------------------------------------------*/
static void StartMainTask(void *pvParameters);
static void StartFAMTask(void *pvParameters);
static void StartConsoleTask(void *pvParameters);  // <──
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

  osCOMP2_Start();

  filledSem = xSemaphoreCreateBinary();
  if (!filledSem) {
    std::cout << "Semaphore creation failed!\n";
    Error_Handler();
  }
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

      /*
      if (count == 100) {
        std::cout << modem.pcminpacked32->spaceavailable()
                  << " bytes available in packed FIFO\n";
        count = 0;
      }
      count += 1;
    */
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
static void cmd_help(const char *);
/* — Command table (NULL-terminated) — */
static const ConsoleCommand cmdTable[] = {
    {"fifo", cmd_fifo, "Show free space in packed FIFO"},
    {"startrx", cmd_startrx, "Enable RX mode"},
    {"stoprx", cmd_stoprx, "Disable RX mode"},
    {"uptime", cmd_uptime, "Show system uptime"},
    {"clk", cmd_clk, "Display system clock frequency"},
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
