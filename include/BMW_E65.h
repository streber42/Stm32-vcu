#ifndef BMW_E65_h
#define BMW_E65_h

/*  This library supports the Powertrain CAN messages for the BMW E65 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "vehicle.h"
#include "my_math.h"

class BMW_E65Class: public Vehicle
{
public:
   void Task10Ms();
   void Task100Ms();
   void Task200Ms();
   void SetRevCounter(int speed) { revCounter = ABS(speed); }
   void SetTemperatureGauge(float temp) { temperature = temp; }
   void DecodeCAN(int, uint32_t* data);
   bool Ready() { return terminal15On; }
   bool Start() { return terminal15On; }
   bool GetGear(Vehicle::gear& outGear);
   void DashOff();
   BMW_E65Class()
   {
      terminal15On = false;
      dashInit = false;
      gear = PARK;
   }

private:
   void SendAbsDscMessages(bool Brake_In);

   bool terminal15On;
   bool dashInit;
   Vehicle::gear gear;
   int revCounter;
   float temperature;
};

#endif /* BMW_E65_h */

