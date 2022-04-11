#include "Can_E46.h"
#include "stm32_can.h"
#include "params.h"

static uint8_t counter_329 = 0;
static uint8_t ABSMsg = 0;
static uint16_t consumption = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

//these messages go out on vehicle can and are specific to driving the E46 instrument cluster etc.

//////////////////////DME Messages //////////////////////////////////////////////////////////
void Can_E46::Msg316(uint16_t speed_input)
{
   // Limit tachometer range from 750 RPMs - 7000 RPMs at max.
   // These limits ensure the vehicle thinks engine is alive and within the
   // max allowable RPM.
   // FIXME: Verify if this is true, or is the Terminal 15 bit which is
   //        more important.
   // Comparison uses ternary operator.
   speed_input = (speed_input < 750) ? 750 : speed_input;
   speed_input = (speed_input > 7000) ? 7000 : speed_input;

   uint8_t rpm_to_can_mult = 64;
   uint8_t rpm_to_can_div = 10;

   uint8_t canRPMlo = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) & 0xFF;
   uint8_t canRPMhi = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) >> 8;

   // Declare data frame array.
   uint8_t bytes[8];

   // Byte 0 - Status - 0x01 is Terminal 15 Status, 0x04 is Traction Control OK
   bytes[0]=0x05;
   // Byte 1 - Torque with all interventions
   bytes[1]=0x00;
   // Byte 2 / 3 "Engine" RPM, RPM * 6.4. 16 bits, Intel LSB (LSB,MSB)
   bytes[2]=canRPMlo;  //RPM LSB
   bytes[3]=canRPMhi;  //RPM MSB
   // Byte 4 - Driver Desired Torque
   bytes[4]=0x00;
   // Byte 5 - Friction Torque
   bytes[5]=0x00;
   // Byte 6 - Various other status bits
   bytes[6]=0x00;
   // Byte 7 - Torque with internal interventions only
   bytes[7]=0x00;

   Can::GetInterface(Param::GetInt(Param::veh_can))->Send(0x316, (uint32_t*)bytes,8);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Can_E46::Msg329(uint16_t tempValue)
{
   //********************temp sense  *******************************
   //  tempValue=analogRead(tempIN); //read Analog pin voltage
   //  The sensor and gauge are not linear.  So if the sensed
   //  Voltage is less than the mid point the Map function
   //  is used to map the input values to output values For the gauge
   //  output values (in decimal):
   //  86 = First visible movment of needle starts
   //  93 = Begining of Blue section
   //  128 = End of Blue Section
   //  169 = Begin Straight up
   //  193 = Midpoint of needle straight up values
   //  219 = Needle begins to move from center
   //  230 = Beginning of Red section
   //  254 = needle pegged to the right
   //  MAP program statement: map(value, fromLow, fromHigh, toLow, toHigh)
   //  if(tempValue < 964){  //if pin voltage < mid point value
   //      tempValue= inverter_status.inverter_temperature;  //read temp from leaf inverter can.
   //      tempValue= map(tempValue,15,80,88,254); //Map to e46 temp gauge
   //  }
   //  else
   //  {
   //      tempValue= map(tempValue,964,1014,219,254); //Map upper half of range
   //  }

   //Can bus data packet values to be sent
   // MSG 0x329

   // Declare data frame array.
   uint8_t bytes[8];

   // Byte 0 - Bits 6-7 Multiplexer ID, Bits 0-5 Data
   bytes[0]=ABSMsg;  //needs to cycle 11,86,d9
   // Byte 1 - Coolant Temperature
   bytes[1]=tempValue; //temp bit tdata
   // Byte 2 - Atmospheric Pressure in mbar
   bytes[2]=0xc5;
   // Byte 3 - Status bits - 0x10 Engine running,
   //          0x08 Coolant Temp > 60 deg C (Warmup bit), Other bits include
   //          cruise control bits.
   bytes[3]=0x00;
   // Byte 4 - Cruise control desired relative torque, 0-99.6% (0-254)
   bytes[4]=0x00;
   // Byte 5 - Driver desired relative torque, 0-99.6% (0-254)
   //          Max of driver or cruise desired value.
   bytes[5]=0x00;
   // Byte 6 - 0x01 Brake Pedal pressed, 0x02 Brake Light Switch error,
   //          0x04 Kickdown, 0x08 Cruise Enabled
   bytes[6]=0x00;
   // Byte 7 - Zero, or -1 (255)
   bytes[7]=0x00;

   counter_329++;
   if(counter_329 >= 22) counter_329 = 0;
   if(counter_329==0) ABSMsg=0x11;
   if(counter_329==8) ABSMsg=0x86;
   if(counter_329==15) ABSMsg=0xd9;

   Can::GetInterface(Param::GetInt(Param::veh_can))->Send(0x329, (uint32_t*)bytes,8);
}

void Can_E46::Msg43F(int8_t gear)
{
   //Can bus data packet values to be sent
   uint8_t bytes[8];
   // Source: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project&p=30055342#post30055342
   // byte 0 = 0x81 //doesn't do anything to the ike
   bytes[0] = 0x81;
   // byte 1 = 0x01 where;
   // 01 = first gear
   // 02= second gear
   // 03 = third gear
   // 04 = fourth gear
   // 05 = D
   // 06 = N
   // 07 = R
   // 08 = P
   // 09 = 5
   // 0A = 6
   switch (gear)
   {
   case -1 /* Reverse */:
      bytes[1] = 0x07;
      break;
   case 0 /* Neutral */:
      bytes[1] = 0x06;
      break;
   case 1 /* Drive */:
      bytes[1] = 0x05;
      break;
   default:
      bytes[1] = 0x08;
      break;
   }

   // byte 2 = 0xFF where;
   // FF = no display
   // 00 = E
   // 39 = M
   // 40 = S
   bytes[2] = 0xFF;

   // byte 3 = 0xFF //doesn't do anything to the ike
   bytes[3] = 0xFF;

   // byte 4 = 0x00 //doesn't do anything to the ike
   bytes[4] = 0x00;

   // byte 5 = 0x80 where;
   // 80 = clears the gear warning picture - all other values bring it on
   bytes[5] = 0x80;

   // byte 6 = 0xFF //doesn't do anything to the ike
   bytes[6] = 0xFF;

   // byte 7 = 0x00 //doesn't do anything to the ike
   bytes[7] = 0xFF;

   Can::GetInterface(Param::GetInt(Param::veh_can))->Send(0x43F, (uint32_t*)bytes,8);
}

void Can_E46::Msg545(int32_t vspeed)
{
    // int z = 0x60; // + y;  higher value lower MPG
    consumption = (consumption + (vspeed/2)) % 65536;

   // Data sent to instrument cluster. Status and slow moving data.
   // Fuel consumption is fuel usage (since start) in uL % 65536.

   //MSG 0x545
   //Can bus data packet values to be sent
   uint8_t bytes[8];

   // Byte 0 - 2 Check Engine, 8 Cruise Enabled , 0x10 EML, 0x40 Gas Cap
   bytes[0]=0x00;
   // Byte 1 - Fuel consumption LSB
   bytes[1]=consumption & 0xFF;
   // Byte 2 - Fuel consumption MSB
   bytes[2]=consumption >> 8;
   // Byte 3 - 0x08 Overheat, Yellow Oil Level 0x02, M3 cluster shift lights
   bytes[3]=0x00;
   // Byte 4 - Oil Temperature
   bytes[4]=0x7E;
   // Byte 5 - Battery light, 0x01.
   bytes[5]=0x10;
   // Byte 6 - Unused
   bytes[6]=0x00;
   // Byte 7 - 0x80 Oil Pressure (Red Oil light), Idle set speed
   bytes[7]=0x18;

   Can::GetInterface(Param::GetInt(Param::veh_can))->Send(0x545, (uint32_t*)bytes,8);
}

void Can_E46::DecodeCAN(int id, uint32_t data[2])
{
    //ASC1 message data 0x153
    /*
        Byte 0 - Bitfield
        Bit 0 - LV_ASC_REQ
        Bit 1 - LV_MSR_REQ
        Bit 2 - LV_ASC_PASV
        Bit 3 - LV_ASC_SW_INT
        Bit 4 - LV_BLS
        Bit 5 - LV_
        Bit 6 - LV_
        Bit 7 - LV_ABS_LED
    Byte 1 - VSS [LSB]
        Bit 0 - LV_ASC_REQ
        Bit 1 - LV_MSR_REQ
        Bit 2 - LV_ASC_PASV
        Bit 3 - VSS [0]
        Bit 4 - VSS [1]
        Bit 5 - VSS [2]
        Bit 6 - VSS [3]
        Bit 7 - VSS [4]
    Byte 2 - VSS [MSB]

        Vehicle speed signal in Km/h
        Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        Min: 0x160 (0 Km/h)

    Byte 3 - MD_IND_ASC

        Torque intervention for ASC function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) max. reductiuon
        Max: 0xFF (99.6094%) no reduction

    Byte 4 - MD_IND_MSR

        Torque intervention for MSR function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) no engine torque increase
        Max: 0xFF (99.6094%) max engine torque increase

    Byte 5 - Unused
    Byte 6 - MD_IND_ASC_LM

        Torque intervention for MSR function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) max. reductiuon
        Max: 0xFF (99.6094%) no reduction

    Byte 7 - ASC ALIVE
    */

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    if (id == 0x153)// ASC1 contains road speed signal.
    {
        //Vehicle speed signal in Km/h
        //Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        //Min: 0x160 (0 Km/h)


        uint16_t road_speed=(((bytes[2]<<8)+(bytes[1])) >> 8);//*0.0625

        Param::SetInt(Param::Veh_Speed,road_speed);
    }
	//	615 comes from the instrument cluster				
    /*
     0x615	B0	AC signal.  Hex 80 when on (10000000)  Other bits say something else (inside temp? system pressure?)				
	         B1	mainly 32 goes to zero once in a while
	         B2	0
	         B3	"Outside Air Temperature: x being temperature in Deg C, (x>=0 deg C,DEC2HEX(x),DEC2HEX(-x)+128) x range min -40  C max 50 C"
          	B4	1 ignition on?
	         B5	0
	         B6	0
	         B7	0
   */	
    if (id == 0x615) 
    {
      Param::SetInt(Param::ACReq, bytes[0] == 0x80);
    }
}
