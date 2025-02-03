/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

//
// Board-specific options need to be defined before HAL.h
//
#if MB(MKS_TINYBEE)
  #if defined(TINYBEE_MASTER)
    #define MAX_EXPANDER_BITS 64  // Master and slave has 6 total HC595 for 48 bits. However the communication protocol for sending 48 bits via i2s requires sending 2 lots of 32 bits. 
  #else
    #define MAX_EXPANDER_BITS 24 // TinyBee has 3 x HC595
  #endif
#endif
