if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  message(WARNING "CMAKE_TOOLCHAIN_FILE is not set! You should configure with -DCMAKE_TOOLCHAIN_FILE=stm32/gcc-arm-none-eabi.cmake")
endif()
# Toolchain & FreeRTOS port/heap

set(FREERTOS_PORT  GCC_ARM_CM4F CACHE STRING "" FORCE)
set(FREERTOS_HEAP  3            CACHE STRING "" FORCE)

add_subdirectory(${CMAKE_SOURCE_DIR}/fontusmodem ${CMAKE_BINARY_DIR}/fontusmodem)
include("stm32_files.cmake")

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE stm32_files fontusModemLib)
# Generate binary and hex images after build
# Use the configured objcopy tool if available
if(CMAKE_OBJCOPY)
  add_custom_command(TARGET ${PROJECT_NAME}
      POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin
      COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.hex
  )
else()
  message(WARNING "CMAKE_OBJCOPY not set; skipping .bin/.hex generation.")
endif()