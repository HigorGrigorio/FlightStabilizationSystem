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

#ifndef Utils_h
#define Utils_h

#include <Arduino.h>

namespace System
{
    namespace Utils
    {
        /**
         * @brief A typed linked iterator to iterate over values in memory.
         *
         * @tparam Ty
         */
        template <typename Ty>
        struct Iterator
        {
        public:
            /**
             * @brief A typed helper Entry.
             *
             */
            struct Entry
            {
                Ty *data_;
                Entry *next_;
            };

            /**
             * @brief Construct a new Iterator object.
             *
             * @param entry Pointer to entry.
             */
            explicit Iterator(Entry *entry)
                    : node_(entry) {}

            /**
             * @brief Get a pointer to data.
             *
             * @return Ty*
             */
            auto put() -> Ty *
            {
                return node_->data_;
            }

            /**
             * @brief Check's if has a value in the current position of memory.
             *
             * @return true
             * @return false
             */
            auto hasValue() -> bool
            {
                return node_;
            }

            /**
             * @brief Check's if has a a next node in the linked list.
             *
             * @return true
             * @return false
             */
            auto hasNext() -> bool
            {
                return node_ && node_->next_;
            }

            /**
             * @brief Get's a next position in linked list.
             *
             * @return Iterator
             */
            auto moveNext() -> Iterator
            {
                node_ = node_->next_;
                return *this;
            }

        private:
            Entry *node_;
        };

        /**
         * @brief Split a string by the delimiter.
         *
         * @param string Pointer to string to split.
         * @param delimiter Delimiter.
         * @return Iterator<String>*
         */
        auto explode(String *string, char delimiter) -> Iterator<String> *;
    } // ! namespace Utils
} // ! namespace System

#endif // ! Utils_h
