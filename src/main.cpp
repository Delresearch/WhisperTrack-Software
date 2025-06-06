/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS (Pure) Includes
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
}

extern "C" {
#include "board.h"
#include "main.h"
}

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
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP 1
#ifdef USE_RTT
#include "SEGGER_RTT.h"

// Custom stream buffer that writes to RTT
class RTTStreamBuffer : public std::streambuf {
  protected:
    int overflow(int c) override {
        if (c != EOF) {
            char z = static_cast<char>(c);
            SEGGER_RTT_Write(0, &z, 1); // Write a single character to RTT channel 0
        }
        return c;
    }

    std::streamsize xsputn(const char *s, std::streamsize n) override {
        SEGGER_RTT_Write(0, s, static_cast<unsigned>(n)); // Write to RTT
        return n;
    }
};

// Redirect `std::cout` to RTT
void redirectStdoutToRTT() {
    static RTTStreamBuffer rttBuf;
    static std::ostream rttOut(&rttBuf);
    std::cout.rdbuf(rttOut.rdbuf());
}

#else // USE_RTT is NOT defined -> Include USB code


  // For huart3 and HAL_UART functions

class UARTStreamBuffer : public std::streambuf
{
protected:
    /**
     * @brief Called when the put area is full or a special character triggers flush.
     *        Here, we transmit every single character directly via UART.
     */
    int overflow(int c) override {
        if (c != EOF) {
            char chr = static_cast<char>(c);

            // Replace '\n' with "\r\n" for typical serial terminal formatting
            if (chr == '\n') {
                const char newline[2] = {'\r', '\n'};
                HAL_UART_Transmit(&huart3,
                                  reinterpret_cast<const uint8_t*>(newline),
                                  2,
                                  HAL_MAX_DELAY);
            } else {
                HAL_UART_Transmit(&huart3,
                                  reinterpret_cast<const uint8_t*>(&chr),
                                  1,
                                  HAL_MAX_DELAY);
            }
        }
        return c;
    }

    /**
     * @brief Writes a sequence of characters. We just call `overflow()` on each char.
     */
    std::streamsize xsputn(const char* s, std::streamsize n) override {
        for (std::streamsize i = 0; i < n; ++i) {
            overflow(s[i]);
        }
        return n;
    }
};

/**
 * @brief Redirect std::cout to UART, transmitting one character at a time.
 */
void redirectStdoutToUART()
{
    static UARTStreamBuffer uartStreamBuf;
    static std::ostream uartStream(&uartStreamBuf);
    std::cout.rdbuf(uartStream.rdbuf());
}


#endif // USE_RTT

#endif // STM32L496xx


/* Definitions ---------------------------------------------------------------*/
#define FAM_TASK_BLOCKING_MS 1000
#define MAX_FFT_SIZE (8192 * 4)
#define CHUNK_SIZE 32

/* Fontus configuration ------------------------------------------------------*/
FontusConfig cfg = {
    hop_values,
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
    12.0f
};

Fontus modem(&cfg);

/* Alignment requirement for DSP operations ----------------------------------*/
int8_t tmpbuf[MAX_FFT_SIZE * 2] __attribute__((aligned(8)));

/* Global Variables (FreeRTOS Task Handles, Queue) ---------------------------*/
TaskHandle_t mainTaskHandle = nullptr;
TaskHandle_t FAM_TaskHandle  = nullptr;
QueueHandle_t xFAMQueue     = nullptr;

/* DSP Instances -------------------------------------------------------------*/
arm_rfft_instance_q15 S15;
arm_rfft_fast_instance_f32 S32;

/* Heap usage tracker --------------------------------------------------------*/
size_t totalHeapUsage = 0;

/* Overload global new/delete to track heap usage ----------------------------*/
void* operator new(size_t size) {
    totalHeapUsage += size;
    void* ptr = calloc(size, sizeof(char));
    if (!ptr) {
        // Stay in an infinite loop if allocation fails
        while (true) {
            // Error handling
        }
    }
    return ptr;
}

void operator delete(void* ptr) noexcept {
    free(ptr);
}

/* Forward Declarations ------------------------------------------------------*/
static void StartMainTask(void* pvParameters);
static void StartFAMTask(void* pvParameters);

/**
 * @brief  The application entry point (Pure FreeRTOS version).
 */
int main(void)
{
#ifdef STM32L496xx
#ifdef USE_RTT
    SEGGER_RTT_Init();
    redirectStdoutToRTT();
#else
    redirectStdoutToUART();
#endif
#endif

    // Hardware / Board initialization
    Board_Init();


    // Print a debug message before starting the scheduler

    // Create queue
    xFAMQueue = xQueueCreate(1, sizeof(uint32_t)); // 1 item, each of size uint32_t
    if (xFAMQueue == NULL) {
        std::cout << "Queue creation failed!\n";
        Error_Handler();
    }

    // Create tasks (pure FreeRTOS)
    BaseType_t result;

    result = xTaskCreate(
        StartMainTask,
        "MainTask",        // Task name
        1024,              // Stack size in words
        nullptr,           // Parameters
        tskIDLE_PRIORITY + 2, // Priority
        &mainTaskHandle
    );
    if (result != pdPASS) {
        std::cout << "Failed to create MainTask!\n";
        Error_Handler();
    }

    result = xTaskCreate(
        StartFAMTask,
        "FAMTask",
        1024,
        nullptr,
        tskIDLE_PRIORITY + 3,
        &FAM_TaskHandle
    );
    if (result != pdPASS) {
        std::cout << "Failed to create FAMTask!\n";
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
static void StartMainTask(void* pvParameters)
{
    (void)pvParameters; // Avoid unused parameter warning

    std::cout << "Hello from StartMainTask!\n";

    // Example signal to send
    uint32_t signal = 1;

    for (;;)
    {
        // Attempt to send a signal to FAM Task
        if (xQueueSend(xFAMQueue, &signal, 0) == pdPASS) {
            std::cout << "Signal sent to FAM Task.\n";
        } else {
            std::cout << "Queue full. Signal not sent.\n";
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief FAM task implementation (pure FreeRTOS).
 */
static void StartFAMTask(void* pvParameters)
{
    (void)pvParameters; // Avoid unused parameter warning

    // Example hardware init call
    osCOMP2_Start();
    std::cout << "COMP2 start!\n";

    uint32_t signal = 0;

    for (;;)
    {
        // Wait (up to FAM_TASK_BLOCKING_MS) for a signal from the Main Task
        if (xQueueReceive(xFAMQueue, &signal, pdMS_TO_TICKS(FAM_TASK_BLOCKING_MS)) == pdTRUE)
        {
            std::cout << "Signal received from Main Task.\n";

            uint32_t dmaBuffer[CHUNK_SIZE];
            size_t bytesRead = 0;
            uint16_t requested_bytes = CHUNK_SIZE * sizeof(uint32_t);

            // Continuously read available data
            while ((bytesRead = osReceive(reinterpret_cast<uint8_t*>(dmaBuffer), requested_bytes)) > 0)
            {
                // Check if there's enough space in the modem buffer
                if (modem.pcminpacked32->spaceavailable() >= CHUNK_SIZE)
                {
                    // Process the newly-received PCM data
                    modem.process_rx(dmaBuffer, CHUNK_SIZE);
                }
                else
                {
                    // Optional: handle case where there's not enough space
                    // e.g., break or discard the data
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            // No signal received within blocking period
            std::cout << "No signal received. Waiting...\n";
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief Error Handler (same as before).
 */
void Error_Handler(void)
{
    while (1)
    {
        // Infinite loop in case of error
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the source file and line where assert_param error occurred.
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    std::cout << "Wrong parameters value: file " << file << " on line " << line << "\n";
}
#endif
