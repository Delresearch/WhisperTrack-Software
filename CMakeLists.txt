cmake_minimum_required(VERSION 3.14)
project(whispertrack)

# 1) Load common settings exactly once
include("${CMAKE_SOURCE_DIR}/cmake/common_settings.cmake")

# 2) Offer build-mode switches
option(STM32 "Build for STM32" OFF)
option(EMU   "Build for emulator" OFF)
# 3) Dispatch into the right subdirectory
if(STM32)
  add_subdirectory(stm32)
elseif(EMU)
  add_subdirectory(emu)
else()
  message(FATAL_ERROR "You must specify -DSTM32=ON or -DEMU=ON")
endif()

# cmake_minimum_required(VERSION 3.14)
# set(CMAKE_CXX_STANDARD 17)
# # Define the build type if not set
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE "Debug")
# endif()

# # Global compiler options
# add_compile_options(-Wall -Wextra -Wpedantic)

# # Tracing configuration
# if(TRACE_ON_ENTER)
#     add_compile_options(-DTRACE_ON_ENTER=1)
# else()
#     add_compile_options(-DTRACE_ON_ENTER=0)
# endif()


# # Build type and optimization settings
# if(PROFILE)
#     set(CMAKE_BUILD_TYPE "Release")
# endif()

# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")

# # Sanitizer options
# if(SANITIZE_ADDRESS)
#     add_compile_options(-fsanitize=address -fsanitize=alignment)
# endif()

# if(SANITIZE_LEAK)
#     add_compile_options(-fsanitize=leak)
# endif()
# include_directories(inc)
# Platform-specific configuration
# if(STM32)
#     ####################################################################
#     # STM32 Build
#     ####################################################################
   
#     message("Building for STM32 platform")
#     if (RTT)
#     add_compile_definitions("USE_RTT")
#     endif()
#     set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" 
#         CACHE FILEPATH "Toolchain file")
#     set(FREERTOS_PORT GCC_ARM_CM4F 
#         CACHE STRING "FreeRTOS port for ARM Cortex-M4 with FPU")
    
#     project(nfwf_app)
    
#     include("cmake/stm32_files.cmake")
#     add_subdirectory(fontusmodem)

#     # Create the main executable for STM32
#     add_executable(${PROJECT_NAME})
#     target_link_libraries(${PROJECT_NAME} stm32_files fontusModemLib)
    
#     # Post-build steps for binary generation
#     add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
#         COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
#         COMMENT "Converting ELF to HEX and BIN formats"
#     )


# elseif(EMU)
#     ####################################################################
#     # Emulator Build
#     ####################################################################
#     message("Building for emulator")
#     project(emu_nfwf_app)
    
#     set(EMU_EXECUTABLE_NAME "${PROJECT_NAME}")
#     set(FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)
    
#     # Generate C++ code from YAML
#     execute_process(COMMAND bash ./codegen.sh)
    
#     # Enable compile commands for indexing
#     set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
#     add_compile_options(-D_WINDOWS_)
    
#     set(FREERTOS_HEAP "3" CACHE STRING "" FORCE)
    
#     # Add fontusmodem library
#     add_subdirectory(fontusmodem)
    
#     # Include EMU files configuration
#     include("cmake/emu_files.cmake")
    
#     # Configure main executable
#     add_executable(${EMU_EXECUTABLE_NAME} src/main.cpp)
#     target_link_libraries(${EMU_EXECUTABLE_NAME} emu_files)
    
#     message("Finished executable: ${EMU_EXECUTABLE_NAME}")

# else()
#     ####################################################################
#     # Error for Missing Target
#     ####################################################################
#     message(FATAL_ERROR "You must specify either -DSTM32=ON or -DEMU=ON.")
# endif()

message("CMake configuration completed.")