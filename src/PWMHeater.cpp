/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
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
#include <PWMHeater.h>
#include <libopencm3/stm32/timer.h>

PWMHeater::PWMHeater() {}

// void PWMHeater::SetTargetTemperature(float temp)
// {
//     (void)temp;
// } // Not supported (yet)?
void PWMHeater::SetPower(uint8_t power, bool HeatReq)
{
    uint16_t heater_temp = AnaIn::GP_analog1.Get();
    Param::SetInt(Param::HeaterTemp, heater_temp);
    if ((Param::GetInt(Param::opmode) == MOD_RUN) && \
         HeatReq && (heater_temp < Param::GetInt(Param::HeatTempMax)) && \
         heater_temp > Param::GetInt(Param::HeatTempMin))
    {
        Param::SetInt(Param::PWMHeatOn,1);
        timer_set_oc_value(TIM3, TIM_OC1, utils::change(power, 0, 100, 0, 3600));
    }
    else
    {
        Param::SetInt(Param::PWMHeatOn,0);
        timer_set_oc_value(TIM3, TIM_OC1, 0);
    }
}
