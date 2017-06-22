cmake_minimum_required(VERSION 3.5)
project(BLDC_ros_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    source/motorshield_test/motorshield_test.ino
    source/bldc/bldc.cpp
    source/bldc/bldc.h
    /opt/arduino/hardware/arduino/avr/cores/arduino/Arduino.h
    /opt/arduino/hardware/arduino/avr/variants/standard/pins_arduino.h
    lib/arduino-softpwm/src/SoftPWM.h
        )

include_directories(lib/arduino-softpwm/src source/bldc /opt/arduino/hardware/arduino/avr/cores/arduino /opt/arduino/hardware/arduino/avr/variants/standard
        lib/ros_lib/ lib/ros_lib/std_msgs lib/ros_lib/ros/
        )

add_executable(BLDC_ros_controller ${SOURCE_FILES})