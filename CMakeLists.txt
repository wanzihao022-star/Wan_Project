cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME QD4310)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}


)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths

)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Libraries/QD4310/QD4310.c
        Libraries/QD4310/QD4310.h
        Libraries/Delay/delay.c
        Libraries/Delay/delay.h
        Application/irqHandlers.c
        Application/irqHandlers.h
        Libraries/PID/pid.c
        Libraries/PID/pid.h
        Libraries/task.h
        Libraries/Vofa/vofa.c
        Libraries/Vofa/vofa.h
        Libraries/HMI/HMI.c
        Libraries/HMI/HMI.h
        Application/MyTask/mytask.c
        Application/MyTask/mytask.h
        Application/Protocol/vision_protocol.c
        Application/Protocol/vision_protocol.h
        Libraries/Laser/laser.c
        Libraries/Laser/laser.h
        Application/App_QD4310_PID/app_QD4310_PID.c
        Application/App_QD4310_PID/app_QD4310_PID.h
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Libraries/QD4310
        Libraries/Delay
        Application
        Libraries/PID
        Libraries
        Libraries/Vofa
        Libraries/HMI
        Application/MyTask
        Application/Protocol
        Libraries/Laser
        Application/App_QD4310_PID
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
target_link_options(${PROJECT_NAME} PRIVATE
        -u _printf_float
)