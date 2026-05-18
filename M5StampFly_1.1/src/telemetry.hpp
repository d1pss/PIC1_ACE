/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <stdint.h>

#define PEER_XX   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7}
#define PING_XX   (uint8_t []){0x77, 0x99, 0x02, 0x09}

//#define PEER_XY   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7}
//#define PING_XY   (uint8_t []){0x77, 99, 0x02, 0x09}




void telemetry(void);
void telemetry_fast(void);

uint8_t make_2station_PING(uint8_t* senddata);
uint8_t make_2station_PEER(uint8_t* senddata);

#endif