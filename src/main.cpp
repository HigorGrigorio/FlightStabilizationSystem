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

#include <Arduino.h>
#include <mEERef.cpp>
#include <Utils.h>
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include <PWM.h>

#define DEBUG 0
#include <debug.h>

enum Kind : u8
{
    WKP = 0,
    WKI = 4,
    WKD = 8,
    WSETPOINT = 12,
    TKP = 16,
    TKI = 20,
    TKD = 24,
    TSETPOINT = 28,
    TOUTPUT,
    WOUTPUT,
    TINPUT,
    WINPUT,
    WING,
    TAIL
};

/**
 * @brief pid values.
 *
 */
double
        wkp,
        wki,
        wkd,
        wsp,
        win,
        wout,
        tkp,
        tki,
        tkd,
        tsp,
        tin,
        tout;

PID *wpid, *tpid;

SoftwareSerial bluetooth(11, 12);

Kind currentEditingPid;

Adafruit_MPU6050 *mpu;
Adafruit_Sensor *acell;
sensors_event_t event;

Servo tservo, wservo, rservo;

System::PWM
        ch1,
        ch2,
        ch3,
        ch4,
        ch5;

unsigned long wdeg, tdeg, rdeg;
long lastC2Read,
    lastC4Read;

/**
 * @brief Update a pid constants in memory and in the eeprom.
 *
 * @param args
 */
void translate(System::Utils::Iterator<String> *);

/**
 * @brief Upadates a constants in the current editing pid.
 *
 */
void updateTunings();

/**
 * @brief Checks if is receiving signal of FS06 Control.
 */
bool checkFS06Signal();

void translate(System::Utils::Iterator<String> *args)
{
    if (!args)
    {
        return;
    }

    auto type = args->put();
    auto dval = args->moveNext().put()->toDouble();
    double *ptr = nullptr;
    Kind vkind;

    // to default values
    if (dval == -1)
    {
        return;
    }

    if (type->indexOf("kp") != -1)
    {
        if (currentEditingPid == WING)
        {
            ptr = &wkp;
            vkind = WKP;
        }
        else
        {
            ptr = &tkp;
            vkind = TKP;
        }
    }
    else if (type->indexOf("ki") != -1)
    {
        if (currentEditingPid == WING)
        {
            ptr = &wki;
            vkind = WKI;
        }
        else
        {
            ptr = &tki;
            vkind = TKI;
        }
    }
    else if (type->indexOf("kd") != -1)
    {
        if (currentEditingPid == WING)
        {
            ptr = &wkd;
            vkind = WKD;
        }
        else
        {
            ptr = &tkd;
            vkind = TKD;
        }
    }
    else if (type->indexOf("setpoint") != -1)
    {
        if (currentEditingPid == WING)
        {
            ptr = &wsp;
            vkind = WSETPOINT;
        }
        else
        {
            ptr = &tsp;
            vkind = TSETPOINT;
        }
    }

    if (ptr && *ptr != dval)
    {
        *ptr = dval;

        // update value on eeprom
        System::mEERef<double>{(u8)vkind}.write(dval);

        // update on pid controls
        updateTunings();
    }
}

void updateTunings()
{
    Print("update -> ");
    if (currentEditingPid == WING)
    {
        Println("Wing: ");
        PrintVarln(wkp);
        PrintVarln(wki);
        PrintVarln(wkd);
        PrintVarln(wsp);
        wpid->SetTunings(wkp, wki, wkd);
    }
    else if (currentEditingPid == TAIL)
    {
        Println("Tail: ");
        PrintVarln(tkp);
        PrintVarln(tki);
        PrintVarln(tkd);
        PrintVarln(tsp);
        tpid->SetTunings(tkp, tki, tkd);
    }
}

bool checkFS06Signal() {
    auto c2 = ch2.read();
    auto c4 = ch4.read();

    if(c2 != lastC2Read || c4 != lastC4Read){
        lastC2Read = c2;
        lastC4Read = c4;
        return true;
    }

    return false;
}

void setup()
{
    wkp = System::mEERef<double>{Kind::WKP}.read();
    wki = System::mEERef<double>{Kind::WKI}.read();
    wkd = System::mEERef<double>{Kind::WKD}.read();
    wsp = System::mEERef<double>{Kind::WSETPOINT}.read();
    tkp = System::mEERef<double>{Kind::WKP}.read();
    tki = System::mEERef<double>{Kind::WKI}.read();
    tkd = System::mEERef<double>{Kind::WKD}.read();
    tsp = System::mEERef<double>{Kind::TSETPOINT}.read();

    Serial.begin(9600);

    while (!Serial)
        ;

    Println("Constants from eeprom...");
    PrintVarln(wkp);
    PrintVarln(wki);
    PrintVarln(wkd);
    PrintVarln(wsp);
    PrintVarln(tkp);
    PrintVarln(tki);
    PrintVarln(tkd);
    PrintVarln(tsp);

    bluetooth.begin(9600);
    while (!bluetooth)
        ;

    // PID tail
    tpid = new PID(&tin, &tout, &tsp, tkp, tki, tkd, P_ON_M, DIRECT);

    tpid->SetMode(AUTOMATIC);
    tpid->SetOutputLimits(-10, 10);
    tpid->SetSampleTime(100);

    // PID wing
    wpid = new PID(&win, &wout, &wsp, wkp, wki, wkd, P_ON_M, DIRECT);

    wpid->SetMode(AUTOMATIC);
    wpid->SetOutputLimits(-10, 10);
    wpid->SetSampleTime(100);

    mpu = new Adafruit_MPU6050();

    while (!(*mpu).begin())
    {
        Println("Adafruit_MPU6050 not founded, wainting for new connection...");
    }

    Println("Adafruit_MPU6050 connected successfully");

    mpu->setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu->setMotionDetectionThreshold(1);
    mpu->setMotionDetectionDuration(20);
    mpu->setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
    mpu->setInterruptPinPolarity(true);
    mpu->setMotionInterrupt(true);

    acell = mpu->getAccelerometerSensor();

    while (!tservo.attached())
    {
        tservo.attach(2);
        Println("Tail servo not attached");
    }

    Println("Tail servo connected successfully");

    while (!wservo.attached())
    {
        wservo.attach(3);
        Println("Wing servo not attached");
    }

    Println("Wing servo connected successfully");

    while (!rservo.attached())
    {
        rservo.attach(4);
        Println("Wing servo not attached");
    }

    Println("Wing servo connected successfully");

    ch1.setPin(5);

    ch2.setPin(6);

    ch3.setPin(7);

    ch4.setPin(8);

    ch5.setPin(9);
}

void loop()
{
    while (bluetooth.available())
    {
        String buff = "";

        while (1)
        {
            char c = bluetooth.read();

            if (c == ';') // the ';' character indicates the final of massage
            {
                break;
            }

            buff.concat(c);

            // wait for next char of argument
            while (!bluetooth.available())
                ;
        }

        if (buff.indexOf("wing") != -1)
        {
            currentEditingPid = WING;
        }
        else if (buff.indexOf("tail") != -1)
        {
            currentEditingPid = TAIL;
        }
        else
        {
            Println(buff);
            translate(System::Utils::explode(&buff, '='));
        }
    }

    PrintVar(ch1.read());
    Print("    ");
    PrintVar(ch2.read());
    Print("    ");
    PrintVar(ch3.read());
    Print("    ");
    PrintVar(ch4.read());
    Print("    ");
    PrintVarln(ch5.read());

    if (ch1.read() > 1500)
    {
        if(!checkFS06Signal()) {
            acell->getEvent(&event);

            win = event.acceleration.y;
            tin = event.acceleration.x;

            if (wpid->Compute()) {
                wdeg = map(wout, -10, 10, 0, 180);
            }

            if (tpid->Compute()) {
                tdeg = map(tout, -10, 10, 0, 180);
            }

            PrintVar(win);
            Print("    ");
            PrintVarln(tin);
        }
        else {
            wdeg = map(lastC2Read, 995, 1995, 0, 180);
            tdeg = map(lastC4Read, 995, 1995, 0, 180);
        }
    }
    else {
        lastC2Read = ch2.read();
        lastC4Read = ch4.read();

        wdeg = map(lastC2Read, 995, 1995, 0, 180);
        tdeg = map(lastC4Read, 995, 1995, 0, 180);
    }



    wservo.write(wdeg);
    tservo.write(tdeg);
    rservo.write(map(ch3.read(), 995, 1995, 0, 180));
}
