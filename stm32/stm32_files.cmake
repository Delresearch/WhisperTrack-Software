# cmake/stm32_files.cmake


add_library(stm32_files INTERFACE)
enable_language(ASM C CXX) 
# Add STM32CubeMX-specific compile definitions
target_compile_definitions(stm32_files INTERFACE
    USE_HAL_DRIVER
    STM32L496xx
    $<$<CONFIG:Debug>:DEBUG>
)
# treat warning as errors

# Add STM32CubeMX-specific include directories
target_include_directories(stm32_files INTERFACE
    ${CMAKE_SOURCE_DIR}/stm32/Core/Inc
    ${CMAKE_SOURCE_DIR}/stm32/RTT
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/portable/GCC/ARM_CM4F
    ${CMAKE_BINARY_DIR}/_deps/cmsis_dsp-src/Include
    ${CMAKE_BINARY_DIR}/_deps/cmsis_5-src/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Common/include
    ${CMAKE_SOURCE_DIR}/Common/nfwf_include
    ${CMAKE_SOURCE_DIR}/fontusmodem/include
)

# Add STM32CubeMX-specific sources
target_sources(stm32_files INTERFACE
    ${CMAKE_SOURCE_DIR}/src/freertos.c
    ${CMAKE_SOURCE_DIR}/stm32/RTT/SEGGER_RTT.c  
    ${CMAKE_SOURCE_DIR}/stm32/RTT/SEGGER_RTT_printf.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/stm32l4xx_it.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/board.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/stm32l4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/stm32l4xx_hal_timebase_tim.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_comp.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
    ${CMAKE_SOURCE_DIR}/stm32/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/system_stm32l4xx.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/croutine.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/event_groups.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/list.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/queue.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/tasks.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/timers.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_4.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/stm32/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/stm32/startup_stm32l496xx.s

)

target_link_directories(stm32_files INTERFACE
)

target_link_libraries(stm32_files INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()
