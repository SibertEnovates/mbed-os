# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-nucleo-g0b0re INTERFACE)

target_sources(mbed-nucleo-g0b0re
    INTERFACE
        PeripheralPins.c
        system_clock.c
)

target_include_directories(mbed-nucleo-g0b0re
    INTERFACE
        .
)

target_link_libraries(mbed-nucleo-g0b0re INTERFACE mbed-stm32g0b0xx)
