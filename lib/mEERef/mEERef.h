/*
 * Copyright © 2022 Flight Stabilization System software.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef FLIGHTSYSTEMCONTROL_MEEREF_H
#define FLIGHTSYSTEMCONTROL_MEEREF_H

#include <Arduino.h>

namespace System {

    template <typename Ty>
    class mEERef
    {
    public:
        mEERef() = delete;

        /**
         * @brief Construct a new mEERef object.
         *
         * @param index position of begin the writing or reading in eeprom.
         */
        explicit mEERef(u8 index);

        /**
         * @brief Read a sizeof(Ty) bytes in the eeprom.
         *
         * @return Ty reinterpreted bytes.
         */
        auto read() -> Ty;

        /**
         * @brief Write a sizeof(Ty) bytes in the eeprom.
         *
         * @param val Value to write in the eeprom.
         */
        auto write(Ty val) -> void;

    private:
        u8 index_;
    };
} // System

#endif //FLIGHTSYSTEMCONTROL_MEEREF_H
