/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2012 Johannes Huebner <contact@johanneshuebner.com>
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin[2];
int Throttle::potmax[2];
s32fp Throttle::brknom;
s32fp Throttle::brknompedal;
s32fp Throttle::brkmax;
s32fp Throttle::brkcruise;
s32fp Throttle::throtmax;
s32fp Throttle::throtmin;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
s32fp Throttle::speedkp;
int Throttle::speedflt;
s32fp Throttle::idleThrotLim;
s32fp Throttle::regenRamp;
s32fp Throttle::throttleRamp;
int Throttle::bmslimhigh;
int Throttle::bmslimlow;
s32fp Throttle::udcmin;
s32fp Throttle::udcmax;
s32fp Throttle::idcmin;
s32fp Throttle::idcmax;
s32fp Throttle::fmax;

int Throttle::speedFiltered;
s32fp Throttle::potnomFiltered;
s32fp Throttle::throttleRamped;

bool Throttle::CheckAndLimitRange(int* potval, int potIdx)
{
    int potMin_local = Throttle::potmax[potIdx] > Throttle::potmin[potIdx] ? Throttle::potmin[potIdx] : Throttle::potmax[potIdx];
    int potMax_local = Throttle::potmax[potIdx] > Throttle::potmin[potIdx] ? Throttle::potmax[potIdx] : Throttle::potmin[potIdx];

    if (((*potval + POT_SLACK) < potMin_local) || (*potval > (potMax_local + POT_SLACK)))
    {
        *potval = potMin_local - 1;
        return false;
    }
    else if (*potval < potMin_local)
    {
        *potval = potMin_local - 1;
    }
    else if (*potval > potMax_local)
    {
        *potval = potMax_local;
    }

    return true;
}

bool Throttle::CheckDualThrottle(int* potval, int pot2val)
{
    int potnom1, potnom2;
    //2nd input running inverse
    if (potmin[1] > potmax[1])
    {
        potnom2 = 100 - (100 * (pot2val - potmax[1])) / (potmin[1] - potmax[1]);
    }
    else
    {
        //         (100 * (0 - 200)) / (1800 - 200)
        //             -20000 / 1600
        //               -200/16
        //                  -25/2
        //                  -12
        potnom2 = (100 * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
    }
    //        (100 * (1024-400)) / (3600 - 400)
    //          62400 / 3200
    //            624/32
    //            156/8
    //            78/4
    //            39/2
    //            19
    potnom1 = (100 * (*potval - potmin[0])) / (potmax[0] - potmin[0]);
    int diff = ABS(potnom2 - potnom1);

    if (diff > 10)
    {
        *potval = potmin[0];
        return false;
    }
    return true;
}

s32fp Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
{
    s32fp potnom;
    s32fp scaledBrkMax = brkpedal ? brknompedal : brkmax;

    // pot2val will never make a difference here
    if (pot2val >= potmin[1])
    {
        potnom = (FP_FROMINT(100) * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
        //Never reach 0, because that can spin up the motor
        scaledBrkMax = -1 + FP_MUL(scaledBrkMax, potnom) / 100;
    }

    if (brkpedal)
    {
        potnom = scaledBrkMax;
    }
    else
    {
        potnom = FP_FROMINT(potval - potmin[0]);
        potnom = FP_DIV(FP_MUL((FP_FROMINT(100) + brknom), potnom),FP_FROMINT(potmax[0] - potmin[0]));
        potnom -= brknom;

        if (potnom < 0)
        {
            potnom = -FP_DIV(FP_MUL(potnom, scaledBrkMax), brknom);
        }
    }

    return potnom;
}

s32fp Throttle::RampThrottle(s32fp potnom)
{
    // min(640, 3200)
    // max(640, -3200)
    potnom = MIN(potnom, throtmax);
    potnom = MAX(potnom, throtmin);
     // 640 >= 0
    if (potnom >= throttleRamped)
    {
    //((potnom < throttleRamped || (throttleRamped + throttleRamp) > potnom) ? potnom : throttleRamped + throttleRamp)
    // 20 < 0 || (0 + 100) > 20 ? 20 : 0 + 100
        // return throttleRamp;
        throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
        potnom = throttleRamped;
    }
    else if (potnom < throttleRamped && potnom > 0)
    {
        throttleRamped = potnom; //No ramping from high throttle to low throttle
    }
    else //potnom < throttleRamped && potnom <= 0
    {
        throttleRamped = MIN(0, throttleRamped); //start ramping at 0
        throttleRamped = RAMPDOWN(throttleRamped, potnom, regenRamp);
        potnom = throttleRamped;
    }
    // return 47;
    return potnom;
}

s32fp Throttle::CalcIdleSpeed(int speed)
{
    int speederr = idleSpeed - speed;
    return MIN(idleThrotLim, speedkp * speederr);
}

s32fp Throttle::CalcCruiseSpeed(int speed)
{
    speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
    int speederr = cruiseSpeed - speedFiltered;

    s32fp potnom = speedkp * speederr;
    potnom = MIN(FP_FROMINT(100), potnom);
    potnom = MAX(brkcruise, potnom);

    return potnom;
}

bool Throttle::TemperatureDerate(s32fp temp, s32fp tempMax, s32fp& finalSpnt)
{
    s32fp limit = 0;

    if (temp <= tempMax)
        limit = FP_FROMINT(100);
    else if (temp < (tempMax + FP_FROMINT(2)))
        limit = FP_FROMINT(50);

    if (finalSpnt >= 0)
        finalSpnt = MIN(finalSpnt, limit);
    else
        finalSpnt = MAX(finalSpnt, -limit);

    return limit < FP_FROMINT(100);
}

/* Currently unused function
void Throttle::BmsLimitCommand(s32fp& finalSpnt, bool dinbms)
{
//  if (dinbms)
//  {
    //    if (finalSpnt >= 0)
    //       finalSpnt = (finalSpnt * bmslimhigh) / 100;
    //    else
    //      finalSpnt = -(finalSpnt * bmslimlow) / 100;
    // }
}
*/

void Throttle::UdcLimitCommand(s32fp& finalSpnt, s32fp udc)
{
    if(udcmin>0)    //ignore if set to zero. useful for bench testing without isa shunt
    {
    if (finalSpnt >= 0)
    {
        s32fp udcErr = udc - udcmin;
        s32fp res = udcErr * 5;
        res = MAX(0, res);
        finalSpnt = MIN(finalSpnt, res);
    }
    else
    {
        s32fp udcErr = udc - udcmax;
        s32fp res = udcErr * 5;
        res = MIN(0, res);
        finalSpnt = MAX(finalSpnt, res);
    }
    }
    else
    {
       finalSpnt = finalSpnt;
    }
}

void Throttle::IdcLimitCommand(s32fp& finalSpnt, s32fp idc)
{
    static s32fp idcFiltered = 0;

    idcFiltered = IIRFILTER(idcFiltered, idc, 4);

    if (finalSpnt >= 0)
    {
        s32fp idcerr = idcmax - idcFiltered;
        s32fp res = idcerr * 5;

        res = MAX(0, res);
        finalSpnt = MIN(res, finalSpnt);
    }
    else
    {
        s32fp idcerr = idcmin - idcFiltered;
        s32fp res = idcerr * 5;

        res = MIN(0, res);
        finalSpnt = MAX(res, finalSpnt);
    }
}

void Throttle::FrequencyLimitCommand(s32fp& finalSpnt, s32fp frequency)
{
    static s32fp frqFiltered = 0;

    frqFiltered = IIRFILTER(frqFiltered, frequency, 4);

    if (finalSpnt > 0)
    {
        s32fp frqerr = fmax - frqFiltered;
        s32fp res = frqerr * 4;

        res = MAX(1, res);
        finalSpnt = MIN(res, finalSpnt);
    }
}
