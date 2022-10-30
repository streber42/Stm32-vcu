#include "custom.h"
#include "params.h"
#include "utils.h"
#include <libopencm3/stm32/timer.h>

void CustomMs200Task() {
    // Turn on AC Relay when AC requested by IHKA
    if (Param::GetBool(Param::ACReq)) {
        utils::GPSet(gpout_roles::AC_RELAY);
        timer_set_oc_value(TIM3,TIM_OC2,5000);
    } else {
        utils::GPClear(gpout_roles::AC_RELAY);
        timer_set_oc_value(TIM3,TIM_OC2,0);
    }
}

void CustomMs100Task() {
    Param::SetInt(Param::WaterTemp,AnaIn::GP_analog2.Get());
    Param::SetInt(Param::HeaterTemp,AnaIn::GP_analog1.Get());
    if ((Param::GetInt(Param::opmode)==MOD_RUN) && Param::GetBool(Param::HeatReq) && (Param::GetInt(Param::HeaterTemp) < Param::GetInt(Param::HeatTempMax)) && (Param::GetInt(Param::HeaterTemp) > Param::GetInt(Param::HeatTempMin))) {
        timer_set_oc_value(TIM3, TIM_OC1, Param::GetInt(Param::HeatPwr));
    } else {
        timer_set_oc_value(TIM3,TIM_OC1,0);
    }
}

void CustomMs10Task() {
   if (Param::GetInt(Param::opmode) == MOD_OFF)
   {
      timer_set_oc_value(TIM3,TIM_OC1,0);
      timer_set_oc_value(TIM3,TIM_OC2,0);
      timer_set_oc_value(TIM3,TIM_OC3,0);
   }
}

void CustomMs1Task() {
    // Put user code here    
}