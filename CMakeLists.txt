cmake_minimum_required(VERSION 3.5)
project(BLDC_ros_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    lib/SoftPWM/SoftPWM.cpp
    lib/SoftPWM/SoftPWM.h
    lib/SoftPWM/SoftPWM_timer.h
    source/motorshield_test/motorshield_test.ino
    source/bldc/bldc.cpp
    source/bldc/bldc.h
    /opt/arduino/hardware/arduino/avr/cores/arduino/Arduino.h
    /opt/arduino/hardware/arduino/avr/variants/standard/pins_arduino.h

        )

include_directories(lib/SoftPWM source/bldc /opt/arduino/hardware/arduino/avr/cores/arduino /opt/arduino/hardware/arduino/avr/variants/standard)

add_executable(BLDC_ros_controller ${SOURCE_FILES})