/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2020 Damien Maguire <info@evbmw.com>
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
#include "stm32_vcu.h"
#include <FreeRTOS.h>
#include <task.h>
#include <libopencm3/cm3/scb.h>

HWREV hwRev; // Hardware variant of board we are running on
static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static bool chargeModeDC = false;
static bool ChgLck = false;
static Can* can;
static Can* can2;
static InvModes targetInverter;
static VehicleModes targetVehicle;
static _chgmodes targetCharger;
static _interface targetChgint;
static uint32_t oldTime;
static uint8_t pot_test;
static uint8_t count_one=0;
static uint8_t ChgSet;
static bool RunChg;
static bool Ampera_Not_Awake=true;
static uint8_t ChgHrs_tmp;
static uint8_t ChgMins_tmp;
static uint16_t ChgDur_tmp;
static uint8_t RTC_1Sec=0;
static uint32_t ChgTicks=0,ChgTicks_1Min=0;
static uint8_t CabHeater,CabHeater_ctrl;
static volatile uint32_t
days=0,
hours=0, minutes=0, seconds=0,
alarm=0;			// != 0 when alarm is pending

// Instantiate Classes
static BMW_E65Class E65Vehicle;
static Can_E39 e39Vehicle;
static Can_VAG vagVehicle;
static GS450HClass gs450Inverter;
static uCAN_MSG rxMessage;
static LeafINV leafInv;
static Can_OI openInv;
static Inverter* selectedInverter = &openInv;
static Vehicle* selectedVehicle = &e39Vehicle;
void setCanFilters();

static void RunChaDeMo()
{
   static uint32_t connectorLockTime = 0;

   chargeMode = true;//General chg boolean. Sets vcu into charge startup.


   /* 1s after entering charge mode, enable charge permission */
   if (Param::GetInt(Param::opmode) == MOD_CHARGE)
   {
      ChaDeMo::SetEnabled(true);
      //here we will need a gpio output to pull the chademo enable signal low
      //main contactor close?
   }

   if (connectorLockTime == 0 && ChaDeMo::ConnectorLocked())
   {
      connectorLockTime = rtc_get_counter_val();
   }
   //10s after locking tell EVSE that we closed the contactor (in fact we have no control). Oh yes we do! Muhahahahaha
   if (Param::GetInt(Param::opmode) == MOD_CHARGE && (rtc_get_counter_val() - connectorLockTime) > 1000)
   {
      //do not do 10 seconds!
      DigIo::gp_out3.Set();//Chademo relay on
      ChaDeMo::SetContactor(true);
      chargeModeDC = true;   //DC charge mode
      Param::SetInt(Param::chgtyp,DCFC);
   }

   if (Param::GetInt(Param::chgtyp) == DCFC)
   {
      int chargeCur = Param::GetInt(Param::CCS_ICmd);
      int chargeLim = Param::GetInt(Param::CCS_ILim);
      chargeCur = MIN(MIN(255, chargeLim), chargeCur);
      ChaDeMo::SetChargeCurrent(chargeCur);
      ChaDeMo::CheckSensorDeviation(Param::GetInt(Param::udc));
   }

   ChaDeMo::SetTargetBatteryVoltage(Param::GetInt(Param::Voltspnt));
   ChaDeMo::SetSoC(Param::Get(Param::CCS_SOCLim));
   Param::SetInt(Param::CCS_Ireq, ChaDeMo::GetRampedCurrentRequest());

   if (chargeMode)
   {
      if (Param::Get(Param::SOC) >= Param::Get(Param::CCS_SOCLim) ||
            Param::GetInt(Param::CCS_ILim) == 0)
      {
         ChaDeMo::SetEnabled(false);
         DigIo::gp_out3.Clear();//Chademo relay off
         chargeMode = false;
      }

      Param::SetInt(Param::CCS_V, ChaDeMo::GetChargerOutputVoltage());
      Param::SetInt(Param::CCS_I, ChaDeMo::GetChargerOutputCurrent());
      ChaDeMo::SendMessages();
   }
   Param::SetInt(Param::CCS_State, ChaDeMo::GetChargerStatus());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void Ms200Task(void)
{
   DigIo::led2_out.Toggle();
   if(chargerClass::HVreq==true) Param::SetInt(Param::hvChg,1);
   if(chargerClass::HVreq==false) Param::SetInt(Param::hvChg,0);
   int opmode = Param::GetInt(Param::opmode);
   Param::SetInt(Param::Day,days);
   Param::SetInt(Param::Hour,hours);
   Param::SetInt(Param::Min,minutes);
   Param::SetInt(Param::Sec,seconds);
   Param::SetInt(Param::ChgT,ChgDur_tmp);
   //  if (DigIo::gp_12Vin.Get()) {
   //      scb_reset_system();
   //  }
   if(ChgSet == 2 && !ChgLck)
   {
      //if in timer mode and not locked out from a previous full charge.
      if(opmode!=MOD_CHARGE)
      {
         if((ChgHrs_tmp==hours) && (ChgMins_tmp==minutes) && (ChgDur_tmp!=0))
            RunChg=true;//if we arrive at set charge time and duration is non zero then initiate charge
         else
            RunChg=false;
      }

      if(opmode==MOD_CHARGE)
      {
         if(ChgTicks != 0)
         {
            ChgTicks--; //decrement charge timer ticks
            ChgTicks_1Min++;
         }

         if(ChgTicks==0)
         {
            RunChg=false; //end charge if still charging once timer expires.
            ChgTicks = (GetInt(Param::Chg_Dur)*300);//recharge the tick timer
         }

         if (ChgTicks_1Min==300)
         {
            ChgTicks_1Min=0;
            ChgDur_tmp--; //countdown minutes of charge time remaining.
         }
      }

   }
   if(ChgSet==0 && !ChgLck) RunChg=true;//enable from webui if we are not locked out from an auto termination
   if(ChgSet==1) RunChg=false;//disable from webui

   selectedVehicle->Task200Ms();

   if(targetCharger == _chgmodes::Volt_Ampera)
   {
      //to be done
   }

   if(targetChgint == _interface::Unused) //No charger interface module used
   {

   }

      if(targetChgint == _interface::Leaf_PDM) //Leaf Gen2/3 PDM charger/DCDC/Chademo
    {
      if (opmode == MOD_CHARGE || opmode == MOD_RUN)  DigIo::inv_out.Set();//inverter and PDM power on if using pdm and in chg mode or in run mode
      if (opmode == MOD_OFF)  DigIo::inv_out.Clear();//inverter and pdm off in off mode. Duh!

      if(opmode != MOD_RUN)                   //only run charge logic if not in run mode.
            {
                if(LeafINV::ControlCharge(RunChg))
                {
                chargeMode = true;   //AC charge mode
                Param::SetInt(Param::chgtyp,AC);
                }
                else if(!LeafINV::ControlCharge(RunChg))
                {
                chargeMode = false;  //no charge mode
                Param::SetInt(Param::chgtyp,OFF);
                }
            }
    }

   if(targetChgint == _interface::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send200msMessages();
   }

   if(targetCharger == _chgmodes::Off)
   {
      chargeMode = false;
   }

   if(targetCharger == _chgmodes::HV_ON)
   {
      if(opmode != MOD_RUN)  chargeMode = true;

   }

   if(targetCharger == _chgmodes::EXT_CAN)
   {


   }

   if(targetCharger == _chgmodes::EXT_DIGI)
   {
      if((opmode != MOD_RUN) && (RunChg))  chargeMode = DigIo::HV_req.Get();//false; //this mode accepts a request for HV via a 12v inputfrom a charger controller e.g. Tesla Gen2/3 M3 PCS etc.
      if(!RunChg) chargeMode = false;

      if(RunChg) DigIo::SP_out.Set();//enable charger digital line. using sp out from gs450h as not used when in charge
      if(!RunChg) DigIo::SP_out.Clear();//disable charger digital line when requested by timer or webui.
   }

   ///////////////////////////////////////
   //Charge term logic
   ///////////////////////////////////////
   /*
   if we are in charge mode and battV >= setpoint and power is <= termination setpoint
       Then we end charge.
   */
   if(opmode==MOD_CHARGE)
   {
      if(Param::GetInt(Param::udc)>=Param::GetInt(Param::Voltspnt) && Param::GetInt(Param::idc)<=Param::GetInt(Param::IdcTerm))
      {
         RunChg=false;//end charge
         ChgLck=true;//set charge lockout flag
      }
   }
   if(opmode==MOD_RUN) ChgLck=false;//reset charge lockout flag when we drive off

   ///////////////////////////////////////



   // if(opmode==MOD_CHARGE) DigIo::gp_out3.Set();//Chademo relay on for testing
   // if(opmode!=MOD_CHARGE) DigIo::gp_out3.Clear();//Chademo relay off for testing

//     count_one++;
// if(count_one==1)    //just a dummy routine that sweeps the pots for testing.
// {
//     pot_test++;
//     DigIo::pot1_cs.Clear();
//     DigIo::pot2_cs.Clear();
//     uint8_t dummy=spi_xfer(SPI3,pot_test);//test
//     dummy=dummy;
//     DigIo::pot1_cs.Set();
//     DigIo::pot2_cs.Set();
//     count_one=0;
// }
}

static void SendInverter100MsMessages(int opmode)
{
   if (opmode == MOD_RUN) selectedInverter->Task100Ms();
   Param::SetInt(Param::tmphs,selectedInverter->GetInverterTemperature());//send leaf temps to web interface
   Param::SetInt(Param::tmpm,selectedInverter->GetMotorTemperature());
   Param::SetInt(Param::InvStat, selectedInverter->GetInverterState()); //update inverter status on web interface
   Param::SetInt(Param::INVudc, selectedInverter->GetInverterVoltage());//display inverter derived dc link voltage on web interface
}

static void SendCharger100MsMessages(int opmode)
{
   if(targetChgint == _interface::Leaf_PDM) //Leaf Gen2 PDM charger/DCDC/Chademo
   {
      if (opmode == MOD_CHARGE)
      {
         //TODO: split off PDM to separate class
         leafInv.Task100Ms(); //send leaf 100ms msgs if we are using the pdm and in charge mode
      }
   }

   if(targetChgint == _interface::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send100msMessages();

      auto LIMmode=i3LIMClass::Control_Charge(RunChg);


      if(LIMmode==i3LIMChargingState::DC_Chg)   //DC charge mode
      {
           if(RunChg) chargeMode = true;// activate charge mode
          chargeModeDC = true;   //DC charge mode
          Param::SetInt(Param::chgtyp,DCFC);
      }

      if(LIMmode==i3LIMChargingState::AC_Chg)
      {
          Param::SetInt(Param::chgtyp,AC);
         if(RunChg) chargeMode = true;// activate charge mode
      }

      if(LIMmode==i3LIMChargingState::No_Chg)
      {
         Param::SetInt(Param::chgtyp,OFF);
         if(chargerClass::HVreq==false) chargeMode = false;//
      }
   }
}

static void SendVehicle100MsMessages()
{
   if (selectedVehicle->Ready())
   {
      selectedVehicle->Task100Ms();
      Param::SetInt(Param::T15Stat,1);
   }
   else
   {
      Param::SetInt(Param::T15Stat,0);
   }
}

static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());
   int opmode = Param::GetInt(Param::opmode);
   utils::SelectDirection(selectedVehicle); 
   utils::ProcessUdc(oldTime, GetInt(Param::speed));
   utils::CalcSOC();

   SendInverter100MsMessages(opmode);
   SendCharger100MsMessages(opmode);
   SendVehicle100MsMessages();

   if (!chargeMode && rtc_get_counter_val() > 100)
   {
      if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
         can2->SendAll();
   }
   int16_t IsaTemp=ISA::Temperature;
   Param::SetInt(Param::tmpaux,IsaTemp);

   chargerClass::Send100msMessages(RunChg);

   if(targetChgint == _interface::Chademo) //Chademo on CAN3
   {
      if(!DigIo::gp_12Vin.Get()) RunChaDeMo(); //if we detect chademo plug inserted off we go ...
   }

   if(targetChgint != _interface::Chademo) //If we are not using Chademo then gp in can be used as a cabin heater request from the vehicle
   {
      Param::SetInt(Param::HeatReq,DigIo::gp_12Vin.Get());
   }
}


static void Ms10Task(void)
{
   int previousSpeed = Param::GetInt(Param::speed);
   int speed = 0;
   float torquePercent;
   int opmode = Param::GetInt(Param::opmode);
   int newMode = MOD_OFF;
   int stt = STAT_NONE;
   int requestedDirection = Param::GetInt(Param::dir);
   ErrorMessage::SetTime(rtc_get_counter_val());

   if(targetChgint == _interface::Leaf_PDM) //Leaf Gen2 PDM charger/DCDC/Chademo
   {
      if (opmode == MOD_CHARGE)
      {
         leafInv.Task10Ms(); //send leaf 10ms msgs if we are using the pdm and in charge mode
      }
   }

   if(targetChgint == _interface::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send10msMessages();
   }

   if (Param::GetInt(Param::opmode) == MOD_RUN)
   {
      torquePercent = utils::ProcessThrottle(previousSpeed);

      //When requesting regen we need to be careful. If the car is not rolling
      //in the same direction as the selected gear, we will actually accelerate!
      //Exclude openinverter here because that has its own regen logic
      if (torquePercent < 0 && Param::GetInt(Param::Inverter) != InvModes::OpenI)
      {
         int rollingDirection = previousSpeed >= 0 ? 1 : -1;

         //When rolling backward while in forward gear, apply POSITIVE torque to slow down backward motion
         //Vice versa when in reverse gear and rolling forward.
         if (rollingDirection != requestedDirection)
         {
            torquePercent = -torquePercent;
         }
      }
      else if (torquePercent >= 0)
      {
         torquePercent *= requestedDirection;
      }
   }
   else
   {
      torquePercent = 0;
      utils::displayThrottle();//just displays pot and pot2 when not in run mode to allow throttle cal
   }

   if (opmode != MOD_OFF)  //send messages only when not in off mode.
   {
      selectedInverter->Task10Ms();
      speed = selectedInverter->GetMotorSpeed(); //set motor rpm on interface
      selectedInverter->SetTorque(torquePercent);//send direction and torque request to inverter
   }

   Param::SetInt(Param::speed, speed);
   utils::GetDigInputs(can);

   selectedVehicle->SetRevCounter(speed);
   selectedVehicle->SetTemperatureGauge(Param::GetFloat(Param::tmphs));

   // Send CAN 2 (Vehicle CAN) messages if necessary for vehicle integration.
   selectedVehicle->Task10Ms();
   //////////////////////////////////////////////////
   //            MODE CONTROL SECTION              //
   //////////////////////////////////////////////////
   float udc = utils::ProcessUdc(oldTime, GetInt(Param::speed));
   stt |= Param::GetInt(Param::potnom) <= 0 ? STAT_NONE : STAT_POTPRESSED;
   stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;

   if (opmode==MOD_OFF && (Param::GetBool(Param::din_start) || selectedVehicle->Start() || chargeMode))//on detection of ign on or charge mode enable we commence prechage and go to mode precharge
   {
      if(chargeMode==false)
      {
         //activate inv during precharge if not oi.
         if(targetInverter != InvModes::OpenI) DigIo::inv_out.Set();//inverter power on but not if we are in charge mode!
      }

      //DigIo::gp_out2.Set();//Negative contactors on
      DigIo::gp_out1.Set();//Coolant pump on
      DigIo::prec_out.Set();//commence precharge
      opmode = MOD_PRECHARGE;
      Param::SetInt(Param::opmode, opmode);
      oldTime=rtc_get_counter_val();
   }

   if(opmode==MOD_PCHFAIL && !selectedVehicle->Start())//use start to reset
   {
      opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   }

   if(opmode==MOD_PCHFAIL && chargeMode)
   {
      //    opmode = MOD_OFF;
      //    Param::SetInt(Param::opmode, opmode);
   }


   /* switch on DC switch if
    * - throttle is not pressed
    * - start pin is high
    * - udc >= udcsw
    * - udc < udclim
    */
   if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
   {
      if (selectedVehicle->Start())
      {
         newMode = MOD_RUN;
      }

      if (chargeMode)
      {
         newMode = MOD_CHARGE;
      }

      stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
   }

   Param::SetInt(Param::status, stt);

   if(opmode == MOD_RUN) //only shut off via ign command if not in charge mode
   {
      if(targetInverter == InvModes::OpenI) DigIo::inv_out.Set();//inverter power on in run only if openi.
      if(!selectedVehicle->Ready()) opmode = MOD_OFF; //switch to off mode via CAS command in an E65
   }

   if(opmode == MOD_CHARGE && !chargeMode) opmode = MOD_OFF; //if we are in charge mode and commdn charge mode off then go to mode off.

   if (newMode != MOD_OFF)
   {
      DigIo::dcsw_out.Set();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }

   if (opmode == MOD_OFF)
   {
      DigIo::inv_out.Clear();//inverter power off
      DigIo::dcsw_out.Clear();
    //   DigIo::gp_out2.Clear();//Negative contactors off
      DigIo::gp_out1.Clear();//Coolant pump off
      DigIo::prec_out.Clear();
      Param::SetInt(Param::dir, 0); // shift to park/neutral on shutdown
      Param::SetInt(Param::opmode, newMode);
      selectedVehicle->DashOff();
   }

   //Cabin heat control
   if((CabHeater_ctrl==1)&& (CabHeater==1)&&(opmode==MOD_RUN))//If we have selected an ampera heater are in run mode and heater not diabled...
   {
      DigIo::gp_out3.Set();//Heater enable and coolant pump on

      if(Ampera_Not_Awake)
      {
         AmperaHeater::sendWakeup();
         Ampera_Not_Awake=false;
      }
      //gp in used as heat request from car (E46 in case of testing). May be poss via CAN also...
      if(!Ampera_Not_Awake) AmperaHeater::controlPower(Param::GetInt(Param::HeatPwr),Param::GetBool(Param::HeatReq));
   }

   if(CabHeater_ctrl==0 || opmode!=MOD_RUN)
   {
      DigIo::gp_out3.Clear();//Heater enable and coolant pump off
      Ampera_Not_Awake=true;
   }
}

static void Ms1Task(void)
{
   selectedInverter->Task1Ms();
   selectedVehicle->Task1Ms();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern void parm_Change(Param::PARAM_NUM paramNum)
{
    // This function is called when the user changes a parameter
    switch (paramNum) {
    case Param::Inverter_CAN:
    case Param::Vehicle_CAN:
    case Param::Shunt_CAN:
    case Param::LIM_CAN:
    case Param::Charger_CAN:
        setCanFilters();
        break;
    case Param::canspeed:
        can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
        can2->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
        break;
    case Param::Inverter:
        selectedInverter->DeInit();
        switch (Param::GetInt(Param::Inverter)) {
          case InvModes::Leaf_Gen1:
            selectedInverter = &leafInv;
            break;
        case InvModes::GS450H:
            selectedInverter = &gs450Inverter;
            gs450Inverter.SetGS450H();
            break;
        case InvModes::Prius_Gen3:
            selectedInverter = &gs450Inverter;
            gs450Inverter.SetPrius();
            break;
        default: //default to OpenI, does the least damage ;)
        case InvModes::OpenI:
            selectedInverter = &openInv;
            break;
        }
        break;
    case Param::Vehicle: {
        switch (Param::GetInt(Param::Vehicle)) {
        case VehicleModes::BMW_E65:
            selectedVehicle = &E65Vehicle;
            break;
        case VehicleModes::BMW_E46:
            selectedVehicle = &e39Vehicle;
            break;
        case VehicleModes::BMW_E39:
            selectedVehicle = &e39Vehicle;
            break;
        case VehicleModes::VAG:
            selectedVehicle = &vagVehicle;
            break;
        }}
        break;
    case Param::Gear:
        gs450Inverter.SetGear(Param::GetInt(Param::Gear));
        break;
    case Param::OilPump:
        gs450Inverter.SetOil(Param::GetInt(Param::OilPump));
        break;

    default:
        break;
    }
    selectedInverter->SetCanInterface(Can::GetInterface(Param::GetInt(Param::Inverter_CAN)));
    selectedVehicle->SetCanInterface(Can::GetInterface(Param::GetInt(Param::Vehicle_CAN)));
    Param::SetInt(Param::inv_can, Param::GetInt(Param::Inverter_CAN));
    Param::SetInt(Param::veh_can, Param::GetInt(Param::Vehicle_CAN));
    Param::SetInt(Param::shunt_can, Param::GetInt(Param::Shunt_CAN));
    Param::SetInt(Param::lim_can, Param::GetInt(Param::LIM_CAN));
    Param::SetInt(Param::charger_can, Param::GetInt(Param::Charger_CAN));

    Throttle::potmin[0] = Param::GetInt(Param::potmin);
    Throttle::potmax[0] = Param::GetInt(Param::potmax);
    Throttle::potmin[1] = Param::GetInt(Param::pot2min);
    Throttle::potmax[1] = Param::GetInt(Param::pot2max);
    Throttle::regenTravel = Param::GetFloat(Param::regentravel);
    Throttle::regenmax = Param::GetFloat(Param::regenmax);
    Throttle::throtmax = Param::Get(Param::throtmax);
    Throttle::throtmin = Param::Get(Param::throtmin);
    Throttle::idcmin = Param::GetFloat(Param::idcmin);
    Throttle::idcmax = Param::GetFloat(Param::idcmax);
    Throttle::udcmin = Param::GetFloat(Param::udcmin); //Leave some room for the notification light
    Throttle::udcmax = Param::GetFloat(Param::udcmax); //Leave some room for the notification light
    Throttle::speedLimit = Param::GetInt(Param::revlim);
    Throttle::regenRamp = 1.0f; //TODO: make parameter
    targetInverter = static_cast<InvModes>(Param::GetInt(Param::Inverter));//get inverter setting from menu
    Param::SetInt(Param::inv, targetInverter);//Confirm mode
    targetVehicle = static_cast<VehicleModes>(Param::GetInt(Param::Vehicle));//get vehicle setting from menu
    Param::SetInt(Param::veh, targetVehicle);//Confirm mode
    targetCharger = static_cast<_chgmodes>(Param::GetInt(Param::chargemodes));//get charger setting from menu
    targetChgint = static_cast<_interface>(Param::GetInt(Param::interface));//get interface setting from menu
    Param::SetInt(Param::Charger, targetCharger);//Confirm mode
    CabHeater = Param::GetInt(Param::Heater);//get cabin heater type
    CabHeater_ctrl = Param::GetInt(Param::Control);//get cabin heater control mode
    if (ChgSet == 1)
    {
        seconds = Param::GetInt(Param::Set_Sec);//only update these params if charge command is set to disable
        minutes = Param::GetInt(Param::Set_Min);
        hours = Param::GetInt(Param::Set_Hour);
        days = Param::GetInt(Param::Set_Day);
        ChgHrs_tmp = GetInt(Param::Chg_Hrs);
        ChgMins_tmp = GetInt(Param::Chg_Min);
        ChgDur_tmp = GetInt(Param::Chg_Dur);
    }
    ChgSet = Param::GetInt(Param::Chgctrl);//0=enable,1=disable,2=timer.
    ChgTicks = (GetInt(Param::Chg_Dur) * 300);//number of 200ms ticks that equates to charge timer in minutes
}


static void CanCallback(uint32_t id, uint32_t data[2]) //This is where we go when a defined CAN message is received.
{
   switch (id)
   {
   case 0x521:
      ISA::handle521(data);//ISA CAN MESSAGE
      break;
   case 0x522:
      ISA::handle522(data);//ISA CAN MESSAGE
      break;
   case 0x523:
      ISA::handle523(data);//ISA CAN MESSAGE
      break;
   case 0x524:
      ISA::handle524(data);//ISA CAN MESSAGE
      break;
   case 0x525:
      ISA::handle525(data);//ISA CAN MESSAGE
      break;
   case 0x526:
      ISA::handle526(data);//ISA CAN MESSAGE
      break;
   case 0x527:
      ISA::handle527(data);//ISA CAN MESSAGE
      break;
   case 0x528:
      ISA::handle528(data);//ISA CAN MESSAGE
      break;
   case 0x108:
      chargerClass::handle108(data);// HV request from an external charger
      break;
   case 0x3b4:
      i3LIMClass::handle3B4(data);// Data msg from LIM
      break;
   case 0x272:
      i3LIMClass::handle272(data);// Data msg from LIM
      break;
   case 0x29e:
      i3LIMClass::handle29E(data);// Data msg from LIM
      break;
   case 0x2b2:
      i3LIMClass::handle2B2(data);// Data msg from LIM
      break;
   case 0x2ef:
      i3LIMClass::handle2EF(data);// Data msg from LIM
      break;

   default:
      selectedInverter->DecodeCAN(id, data);
      selectedVehicle->DecodeCAN(id, data);

      break;
   }
}


static void ConfigureVariantIO()
{
   hwRev = HW_REV1;
   Param::SetInt(Param::hwver, hwRev);

   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   AnaIn::Start();
}


extern "C" void tim3_isr(void)
{
   scheduler->Run();
}


extern "C" void exti15_10_isr(void)    //CAN3 MCP25625 interruppt
{
   uint32_t canData[2];
   if(CANSPI_receive(&rxMessage))
   {
      canData[0]=(rxMessage.frame.data0 | rxMessage.frame.data1<<8 | rxMessage.frame.data2<<16 | rxMessage.frame.data3<<24);
      canData[1]=(rxMessage.frame.data4 | rxMessage.frame.data5<<8 | rxMessage.frame.data6<<16 | rxMessage.frame.data7<<24);
   }
   //can cast this to uint32_t[2]. dont be an idiot! * pointer
   CANSPI_CLR_IRQ();   //Clear Rx irqs in mcp25625
   exti_reset_request(EXTI15); // clear irq

   if(rxMessage.frame.id==0x108) ChaDeMo::Process108Message(canData);
   if(rxMessage.frame.id==0x109) ChaDeMo::Process109Message(canData);
   //DigIo::led_out.Toggle();
}

extern "C" void rtc_isr(void)
{
    /* The interrupt flag isn't cleared by hardware, we have to do it. */
    rtc_clear_flag(RTC_SEC);    //This will fire every 10ms so we need to count to 100 to get a 1 sec tick.
    RTC_1Sec++;

    if (RTC_1Sec == 100)
    {
        RTC_1Sec = 0;
        if (++seconds >= 60) {
            ++minutes;
            seconds -= 60;
        }
        if (minutes >= 60) {
            ++hours;
            minutes -= 60;
        }
        if (hours >= 24) {
            ++days;
            hours -= 24;
        }
    }
}

void setCanFilters() {
    Can* inverter_can = Can::GetInterface(Param::GetInt(Param::inv_can));
    Can* vehicle_can = Can::GetInterface(Param::GetInt(Param::veh_can));
    Can* shunt_can = Can::GetInterface(Param::GetInt(Param::shunt_can));
    Can* lim_can = Can::GetInterface(Param::GetInt(Param::lim_can));
    Can* charger_can = Can::GetInterface(Param::GetInt(Param::charger_can));
    inverter_can->RegisterUserMessage(0x1DA);//Leaf inv msg
    inverter_can->RegisterUserMessage(0x55A);//Leaf inv msg
    inverter_can->RegisterUserMessage(0x679);//Leaf obc msg
    inverter_can->RegisterUserMessage(0x390);//Leaf obc msg
    inverter_can->RegisterUserMessage(0x190);//Open Inv Msg
    inverter_can->RegisterUserMessage(0x19A);//Open Inv Msg
    inverter_can->RegisterUserMessage(0x1A4);//Open Inv Msg
    shunt_can->RegisterUserMessage(0x521);//ISA MSG
    shunt_can->RegisterUserMessage(0x522);//ISA MSG
    shunt_can->RegisterUserMessage(0x523);//ISA MSG
    shunt_can->RegisterUserMessage(0x524);//ISA MSG
    shunt_can->RegisterUserMessage(0x525);//ISA MSG
    shunt_can->RegisterUserMessage(0x526);//ISA MSG
    shunt_can->RegisterUserMessage(0x527);//ISA MSG
    shunt_can->RegisterUserMessage(0x528);//ISA MSG
    lim_can->RegisterUserMessage(0x3b4);//LIM MSG
    lim_can->RegisterUserMessage(0x29e);//LIM MSG
    lim_can->RegisterUserMessage(0x2b2);//LIM MSG
    lim_can->RegisterUserMessage(0x2ef);//LIM MSG
    lim_can->RegisterUserMessage(0x272);//LIM MSG

    // Set up CAN 2 (Vehicle CAN) callback and messages to listen for.
    vehicle_can->RegisterUserMessage(0x130);//E65 CAS
    vehicle_can->RegisterUserMessage(0x192);//E65 Shifter
    charger_can->RegisterUserMessage(0x108);//Charger HV request
    vehicle_can->RegisterUserMessage(0x153);//E39/E46 ASC1 message    
}
/*
 * Handler in case our application overflows the stack
 */
void vApplicationStackOverflowHook(
	TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused))) {

	for (;;);
}

static void rtos_Ms1Task(void* args __attribute__((unused))) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        Ms1Task();
    }
}
static void rtos_Ms10Task(void *args __attribute__((unused))) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        Ms10Task();
    }
}
static void rtos_Ms100Task(void *args __attribute__((unused))) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        Ms100Task();
    }
}
static void rtos_Ms200Task(void *args __attribute__((unused))) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(200);
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        Ms200Task();
    }
}

static void rtos_term_Run(void *args __attribute__((unused))) {
   extern const TERM_CMD TermCmds[];
    Terminal t(USART3, TermCmds, false);
    for (;;) {
      t.Run();
      vTaskDelay(1);
    }
}

extern "C" int main(void)
{
   bool remapCan1 = false;

    clock_setup();
    rtc_setup();
    ConfigureVariantIO();
   #ifdef TEST_P107
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,AFIO_MAPR_CAN1_REMAP_PORTB|AFIO_MAPR_CAN2_REMAP);//32f107
   remapCan1 = true;
   #else
   // gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP);//remap usart 3 to PC10 and PC11 for VCU HW
   //  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_CAN2_REMAP | AFIO_MAPR_TIM1_REMAP_FULL_REMAP);//32f107
   #endif
    usart2_setup();//TOYOTA HYBRID INVERTER INTERFACE
    nvic_setup();
    parm_load();
    spi2_setup();
    spi3_setup();
    DigIo::inv_out.Clear();//inverter power off during bootup
    DigIo::mcp_sby.Clear();//enable can3

    Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));
    Can c2(CAN2, (Can::baudrates)Param::GetInt(Param::canspeed));

    // Set up CAN 1 callback and messages to listen for
    c.SetReceiveCallback(CanCallback);
    c2.SetReceiveCallback(CanCallback);
    setCanFilters();


    can = &c;
    can2 = &c2;

   parm_Change(Param::PARAM_LAST);
   parm_Change(Param::Inverter); //Set loaded inverter

   //  CANSPI_Initialize();// init the MCP25625 on CAN3
   //  CANSPI_ENRx_IRQ();  //init CAN3 Rx IRQ

    xTaskCreate(rtos_Ms1Task, "Ms1Task",100,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(rtos_Ms10Task, "Ms10Task",100,NULL,configMAX_PRIORITIES-2,NULL);
    xTaskCreate(rtos_Ms100Task, "Ms100Task",100,NULL,configMAX_PRIORITIES-3,NULL);
    xTaskCreate(rtos_Ms200Task, "Ms200Task",100,NULL,configMAX_PRIORITIES-4,NULL);
    xTaskCreate(rtos_term_Run, "TermTask",300,NULL,configMAX_PRIORITIES-5,NULL);


    // ISA::initialize();//only call this once if a new sensor is fitted. Might put an option on web interface to call this....
    //  DigIo::prec_out.Set();//commence precharge
    Param::SetInt(Param::version, 4); //backward compatibility

    // Start RTOS Task scheduler
	vTaskStartScheduler();

	for (;;);
    return 0;
}
