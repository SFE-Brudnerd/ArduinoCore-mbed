/*
 * Copyright (c) 2019 Arduino SA
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* MBED TARGET LIST: SFE_NINAB306_THING_PLUS */

#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

///> define macro producing for example Px_y = NRF_GPIO_PIN_MAP(x, y)
#define PinDef(port_num, pin_num) P##port_num##_##pin_num = NRF_GPIO_PIN_MAP(port_num, pin_num)


typedef enum {
    PinDef(0 , 0), // P0_0 = 0...
    PinDef(0 , 1),
    PinDef(0 , 2),
    PinDef(0 , 3),
    PinDef(0 , 4),
    PinDef(0 , 5),
    PinDef(0 , 6),
    PinDef(0 , 7),
    PinDef(0 , 8),
    PinDef(0 , 9),
    PinDef(0 , 10),
    PinDef(0 , 11),
    PinDef(0 , 12),
    PinDef(0 , 13),
    PinDef(0 , 14),
    PinDef(0 , 15),
    PinDef(0 , 16),
    PinDef(0 , 17),
    PinDef(0 , 18),
    PinDef(0 , 19),
    PinDef(0 , 20),
    PinDef(0 , 21),
    PinDef(0 , 22),
    PinDef(0 , 23),
    PinDef(0 , 24),
    PinDef(0 , 25),
    PinDef(0 , 26),
    PinDef(0 , 27),
    PinDef(0 , 28),
    PinDef(0 , 29),
    PinDef(0 , 30),
    PinDef(0 , 31),

    PinDef(1 , 0), //P1_1 = 32...
    PinDef(1 , 1),
    PinDef(1 , 2),
    PinDef(1 , 3),
    PinDef(1 , 4),
    PinDef(1 , 5),
    PinDef(1 , 6),
    PinDef(1 , 7),
    PinDef(1 , 8),
    PinDef(1 , 9),
    PinDef(1 , 10),
    PinDef(1 , 11),
    PinDef(1 , 12),
    PinDef(1 , 13),
    PinDef(1 , 14),
    PinDef(1 , 15),

    // Port0
    p0  = P0_0,
    p1  = P0_1,
    p2  = P0_2,
    p3  = P0_3,
    p4  = P0_4,
    p5  = P0_5,
    p6  = P0_6,
    p7  = P0_7,
    p8  = P0_8,
    p9  = P0_9,
    p10 = P0_10,
    p11 = P0_11,
    p12 = P0_12,
    p13 = P0_13,
    p14 = P0_14,
    p15 = P0_15,
    p16 = P0_16,
    p17 = P0_17,
    p18 = P0_18,
    p19 = P0_19,
    p20 = P0_20,
    p21 = P0_21,
    p22 = P0_22,
    p23 = P0_23,
    p24 = P0_24,
    p25 = P0_25,
    p26 = P0_26,
    p27 = P0_27,
    p28 = P0_28,
    p29 = P0_29,
    p30 = P0_30,
    p31 = P0_31,
    
    // Port1
    p32 = P1_0,
    p33 = P1_1,
    p34 = P1_2,
    p35 = P1_3,
    p36 = P1_4,
    p37 = P1_5,
    p38 = P1_6,
    p39 = P1_7,
    p40 = P1_8,
    p41 = P1_9,
    p42 = P1_10,
    p43 = P1_11,
    p44 = P1_12,
    p45 = P1_13,
    p46 = P1_14,
    p47 = P1_15,

    // IO definitions implemented from NINA-B306 datasheet
    IO1 = P0_13,
    IO2 = P0_14,
    IO3 = P0_15,
    IO4 = P0_16,
    IO5 = P0_24,
    IO7 = P0_25,
    IO17 = P0_28,
    IO18 = P0_2,
    IO20 = P0_31,
    IO21 = P1_12,
    IO22 = P1_13,
    IO23 = P0_29,
    IO24 = P0_30,
    IO25 = P0_4,
    IO27 = P0_5,
    IO34 = P1_8,
    IO35 = P1_1,
    IO37 = P1_3,
    IO38 = P1_10,
    IO39 = P1_11,
    IO40 = P1_15,
    IO41 = P1_14,
    IO42 = P0_26,
    IO43 = P0_6,
    IO44 = P0_27,
    IO48 = P0_21,
    IO50 = P0_20,
    IO51 = P0_17,
    IO52 = P0_19,
 
    RX_PIN_NUMBER  = IO5,
    TX_PIN_NUMBER  = IO7,
    
    LED_BUILTIN = IO41,
    LED_RGB = IO34,
    
    // mBed interface Pins
    CONSOLE_TX = TX_PIN_NUMBER,
    CONSOLE_RX = RX_PIN_NUMBER,
#if defined(MBED_CONF_TARGET_STDIO_UART_TX)
    STDIO_UART_TX   = MBED_CONF_TARGET_STDIO_UART_TX,
#else
    STDIO_UART_TX   = CONSOLE_TX,
#endif
#if defined(MBED_CONF_TARGET_STDIO_UART_RX)
    STDIO_UART_RX   = MBED_CONF_TARGET_STDIO_UART_RX,
#else
    STDIO_UART_RX   = CONSOLE_RX,
#endif

    SPI_PSELMOSI0 = IO50,
    SPI_PSELMISO0 = IO48,
    SPI_PSELSS0   = IO51,
    SPI_PSELSCK0  = IO52,

    SPIS_PSELMOSI = IO50,
    SPIS_PSELMISO = IO48,
    SPIS_PSELSS   = IO51,
    SPIS_PSELSCK  = IO52,

    I2C_SDA0 = IO43,
    I2C_SCL0 = IO42,

#ifndef ARDUINO_ARCH_MBED
    // Digital Pins
    D0 = IO5,
    D1 = IO7,
    D2 = IO38,
    D3 = IO43,
    D4 = IO42,
    D5 = IO35,
    D6 = IO25,
    D9 = IO4,
    D10 = IO3,
    D11 = IO2,
    D12 = IO1,
    D13 = IO44,
  
    // Analog Pins
    A0 = IO24,
    A1 = IO23,
    A2 = IO27,
    A3 = IO20,
    A4 = IO18,
    A5 = IO17,
#endif

    G0 = IO44,
    G1 = IO1,
    G2 = IO2,
    G3 = IO3,
    G4 = IO4,
    G5 = IO25,
    G6 = IO35,

    AIN0 = IO24,
    AIN1 = IO23,
    AIN2 = IO27,
    AIN3 = IO20,
    AIN4 = IO18,
    AIN5 = IO17,

    // Not connected
    NC = (int)0xFFFFFFFF,

    STDIO_UART_RTS = NC,
    STDIO_UART_CTS = NC,

    // Qwiic Port power enable
    QWIIC_PEN = IO40,

    // Fuel Gauge alert pin
    BATT_ALERT = IO39,

    // IMU Interrupts
    IMU_INT1 = IO21,
    IMU_INT2 = IO22

} PinName;

typedef enum {
    PullNone = 0,
    PullDown = 1,
    PullUp = 3,
    PullDefault = PullUp
} PinMode;

#ifdef __cplusplus
}
#endif

#endif
