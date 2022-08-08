#ifndef HEATER_H
#define HEATER_H


#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "CANSPI.h"
#include "digio.h"
#include "utils.h"
#include <libopencm3/stm32/timer.h>

class AmperaHeater
{
   public:
    static void sendWakeup();
    static void controlPower(uint16_t heatPwr, bool heatReq);
    protected:

    private:


};

class VWHeater
{
   public:


    protected:

    private:


};

class PWMHeater
{
    public:
      static void enable_pwm();
      static void disable_pwm();
      static void set_duty_cycle(float cycle);
      static void Task10Ms();


};
#endif // HEATER_H
