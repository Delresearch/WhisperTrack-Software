# --- common_settings.cmake ---
# Minimum CMake version requirement
cmake_minimum_required(VERSION 3.14)

# C++ standard, warning flags, include dirs, etc.
set(CMAKE_CXX_STANDARD 17 CACHE STRING "" FORCE)

# Define the build type if not set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Global compiler options
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Build type and optimization settings
option(PROFILE "Enable profiling build" OFF)
if(PROFILE)
    set(CMAKE_BUILD_TYPE "Release")
endif()


include_directories(${CMAKE_SOURCE_DIR}/inc)

# Tracing / sanitizers
option(TRACE_ON_ENTER "Enable TRACE_ON_ENTER" OFF)
add_compile_definitions($<$<BOOL:${TRACE_ON_ENTER}>:TRACE_ON_ENTER=1>)

option(SANITIZE_ADDRESS "AddressSanitizer" OFF)
if(SANITIZE_ADDRESS)
    add_compile_options(-fsanitize=address -fsanitize=alignment)
endif()

option(SANITIZE_LEAK "LeakSanitizer" OFF)
if(SANITIZE_LEAK)
    add_compile_options(-fsanitize=leak)
endif()


# codegen, compile-commands, Windows-defs…
execute_process(
    COMMAND bash codegen.sh
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE _codegen_result
    OUTPUT_QUIET
    ERROR_QUIET
)
if(NOT _codegen_result EQUAL 0)
    message(FATAL_ERROR "codegen.sh failed with exit code ${_codegen_result}")
endif()
