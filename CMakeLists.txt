#
# Servo Timing Test
#
# Author: LikeSmith
# Date: MARCH 2023
#
# CMake file for user space processes for the servo timer test
#

cmake_minimum_required(VERSION 3.22)
project(Servos VERSION 0.1)

# Compiler Settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED true)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Executable Targets
add_executable(servo_user_test src/servo_user_test.c)
add_executable(servo_kernel_test src/servo_kernel_test.c)

# Add target links and includes
target_link_libraries(servo_user_test PUBLIC pthread)
target_link_libraries(servo_kernel_test PUBLIC pthread)
