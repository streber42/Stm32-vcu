#ifndef Can_E39_h
#define Can_E39_h

/*  This library supports the Powertrain CAN messages for the BMW E39 for driving dash gauges, putting out malf lights etc

*/

#include <stdint.h>
#include "vehicle.h"


class Can_E39: public Vehicle
{

public:
   void Task10Ms();
   void SetRevCounter(int s) { speed = s; }
   void SetTemperatureGauge(float temp);
   void DecodeCAN(int id, uint32_t* data);
   bool Ready();
   bool Start();

private:
   void Msg316();
   void Msg329();
   void Msg43F();
   void Msg545();

   uint16_t speed;
};

#endif /* Can_E39_h */

