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

#include <mEERef.h>
#include <EEPROM.h>

template <typename Ty>
System::mEERef<Ty>::mEERef(u8 index)
        : index_(index)
{
}

template <typename Ty>
auto System::mEERef<Ty>::read() -> Ty
{
    u8 *buff = new u8[sizeof(Ty) / sizeof(u8)];

    for (u8 i = index_, j = 0;
         j < sizeof(Ty) / sizeof(u8);
         i++, j++)
    {
        buff[j] = EEPROM.read(i);
    }

    return *reinterpret_cast<Ty *>(buff);
}

template <typename Ty>
auto System::mEERef<Ty>::write(Ty val) -> void
{
    u8 *buff = reinterpret_cast<u8 *>(&val);

    for (u8 i = index_, j = 0;
         j < sizeof(Ty) / sizeof(u8);
         i++, j++)
    {
        EEPROM.write(i, buff[j]);
    }
}
