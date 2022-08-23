/**
 * @file PWM.cpp
 * @author Higor Grigorio dos Santos (higorgrigorio@gmail.com)
 * @brief implementation of PWM filter class
 * @version 0.1
 * @date 2022-07-14
 *
 * @copyright Copyright (c) 2022 Higor Grigorio dos Santos
 *
 */

#include <PWM.h>

unsigned long System::PWM::read() noexcept
{
    unsigned long reading = pulseIn(pin_, HIGH);

    if (useCalibrate_)
    {
        if (handle_)
        {
            if (reading > (*handle_ + off_) || reading < (*handle_ - off_))
            {
                *handle_ = reading;
            }
        }
        else
        {
            if (reading > (value_ + off_) || reading < (value_ - off_))
            {
                value_ = reading;
            }
        }
    }
    else
    {
        if (handle_)
        {
            *handle_ = reading;
        }
        else
        {
            value_ = reading;
        }
    }

    return value();
}

void System::PWM::calculateOffSet(int sampleSize) noexcept
{
    if (sampleSize > 500)
    {
        return;
    }

    int sum = 0, average = 0, summation = 0;
    int *sample = new int[sampleSize];

    for (int i = 0; i < sampleSize; i++)
    {
        sample[i] = read();
        sum += sample[i];
    }

    average = abs(sum / sampleSize);

    for (int i = 0; i < sampleSize; i++)
    {
        summation = pow((sample[i] - average), 2);
    }

    delete sample;

    // sum .5 to round up.
    off_ = (sqrt(summation / (sampleSize)) * .65) + 0.5;
    useCalibrate_ = true;
}

void System::PWM::attach(unsigned long *pointer) noexcept
{
    handle_ = pointer;
}

bool System::PWM::attached() noexcept
{
    return handle_ != nullptr;
}

unsigned long *System::PWM::detach() noexcept
{
    auto temp = handle_;
    handle_ = nullptr;
    return temp;
}

unsigned long *System::PWM::put() noexcept
{
    return handle_;
}

unsigned long System::PWM::value() noexcept
{
    return   handle_ ? *handle_ : value_;
}

void System::PWM::setPin(u8 pin) noexcept
{
    pin_ = pin;
}

u8 System::PWM::getPin() noexcept
{
    return pin_;
}

unsigned long System::PWM::getOffset() noexcept
{
    return off_;
}

void System::PWM::setOffset(unsigned long value) noexcept
{
    off_ = value;
    useCalibrate_ = true;
}
