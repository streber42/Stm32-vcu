#include "GS450H.h"
#include "hwinit.h"
#include "temp_meas.h"
#include <libopencm3/stm32/timer.h>
#include "anain.h"
#include "my_math.h"
#include "utils.h"

#define  LOW_Gear  0
#define  HIGH_Gear  1
#define  AUTO_Gear  2


static uint8_t htm_state = 0;
static uint8_t inv_status = 1;//must be 1 for gs450h
uint16_t counter;
static uint16_t htm_checksum;
static uint8_t frame_count;
static int16_t mg1_torque, mg2_torque, speedSum;
bool statusInv = 0;
int16_t GS450HClass::dc_bus_voltage;
int16_t GS450HClass::temp_inv_water;
int16_t GS450HClass::temp_inv_inductor;
int16_t GS450HClass::mg1_speed;
int16_t GS450HClass::mg2_speed;


//80 bytes out and 100 bytes back in (with offset of 8 bytes.
static uint8_t mth_data[120];
static uint8_t htm_data_setup[100] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1 };
static uint8_t htm_data[100] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0 };

#if 0
// Not currently used
static uint8_t htm_data_setup_auris[100] = { 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x01 };
#endif

uint8_t htm_data_init[7][100]=
{
   {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
   {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,1,0,0,0,72,7},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
   {0,30,0,0,0,0,0,18,0,154,250,0,0,255,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,255,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,255,4,73,25,60,246,52,8,0,0,255,0,0,0,138,0,0,0,168,0,0,0,3,0,0,0,70,9},
   {0,30,0,2,0,0,0,18,0,154,250,0,0,16,0,97,0,0,0,0,0,0,200,249,56,6,165,0,136,0,63,0,16,0,0,0,63,0,16,0,3,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,16,0,75,12,45,248,21,6,0,0,16,0,0,0,202,0,211,0,16,0,0,0,134,16,0,0,130,10}
};


/////////regen data///////////////
int16_t pedalmap_drive[11][6] = {     //torque 0-3500 (full scale for MG2)
{350, 	700, 	1050, 	1575, 	2450, 	3500},
{175, 	525, 	1050, 	1575, 	2450, 	3500},
{0, 	350, 	875, 	1575, 	2450, 	3500},
{-350, 	105, 	765, 	1487, 	2406, 	3500},
{-525, 	0, 	656, 	1400, 	2362, 	3500},
{-525, 	-35, 	546, 	1312, 	2318, 	3500},
{-392, 	-70, 	437, 	1225, 	2275, 	3500},
{-312, 	-105, 	328, 	1137, 	2231, 	3500},
{-259, 	-140, 	218, 	1050, 	2187, 	3500},
{-221, 	-175, 	109, 	962, 	2143, 	3500},
{-193, 	-140, 	0, 	875, 	2100, 	3500} };

int16_t pedalmap_reverse[5][6] = { //torque 0-3500 (full scale for MG2)
{700, 	525, 	350, 	175, 	87, 	0},
{350, 	-350, 	-525, 	-700, 	-875, 	-1050},
{0, 	-700, 	-1050, 	-1330, 	-1575, 	-1750},
{-350, 	-700, 	-1050, 	-1330, 	-1575, 	-1750},
{-700, 	-875, 	-1120, 	-1330, 	-1575, 	-1750} };

int16_t speedrange_drive[11] = //rpm
{ -3500, 	-1750, 	0, 	1750, 	3500, 	5250, 	7000, 	8750, 	10500, 	12250, 	14000 };

int16_t speedrange_reverse[5] = //rpm
{ -3500, 	-1750, 	0, 	1750, 	3500 };

short GS450HClass::get_torque()
{
    s32fp ThrotVal = utils::GetUserThrottleCommand(); // GetUserThrottleCommand
    // int ThrotRange = parameters.Max_throttleVal - parameters.Min_throttleVal; //full range of min-max throttle params
    int16_t torque = 0, map_x, map_y;
    uint8_t pedal_index, speed_index;
    // uint32_t pedal_range[6] = { parameters.Min_throttleVal, 	parameters.Min_throttleVal + ThrotRange / 5, 	parameters.Min_throttleVal + 2 * ThrotRange / 5, 	parameters.Min_throttleVal + 3 * ThrotRange / 5, 	parameters.Min_throttleVal + 4 * ThrotRange / 5, 	parameters.Max_throttleVal };
    s32fp pedal_range[6] = {FP_FROMINT(0), FP_FROMINT(20), FP_FROMINT(40),FP_FROMINT(60), FP_FROMINT(80),FP_FROMINT(100)};
    int16_t mg2_speed_temp = mg2_speed;
    if (Param::GetInt(Param::dir) == 1) {
        if (mg2_speed_temp < speedrange_drive[0]) {
            mg2_speed_temp = speedrange_drive[0]; // force min speed if speed below expected range
            //SerialDEBUG.print("Below speed map range");
        }
        pedal_index = utils::change(ThrotVal, pedal_range[0], pedal_range[5], 0, 5);
        speed_index = utils::change(mg2_speed_temp, speedrange_drive[0], speedrange_drive[10], 0, 10);
        //SerialDEBUG.print("Pedal map - Pedal/Speed "); SerialDEBUG.print(pedal_index); SerialDEBUG.print("/"); SerialDEBUG.println(speed_index);

        if (pedal_index >= 5 && speed_index >= 10) {
            return (pedalmap_drive[10][5]); // pedal and speed maxed out
            //SerialDEBUG.println("Pedal map - Pedal & Speed maxed out");
        }
        if (pedal_index >= 5) {
            return (utils::change(mg2_speed_temp, speedrange_drive[speed_index], speedrange_drive[speed_index + 1], pedalmap_drive[speed_index][5], pedalmap_drive[speed_index + 1][5])); // pedal maxed out
            //SerialDEBUG.println("Pedal map - Pedal maxed out");;
        }
        if (speed_index >= 10) {
            return (utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[10][pedal_index], pedalmap_drive[10][pedal_index + 1])); // speed maxed out
            //SerialDEBUG.println("Pedal map - Speed maxed out");
        }

        map_x = utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[speed_index][pedal_index], pedalmap_drive[speed_index][pedal_index + 1]);
        map_y = utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[speed_index + 1][pedal_index], pedalmap_drive[speed_index + 1][pedal_index + 1]);
        //SerialDEBUG.print("Pedal map - Interp x/y "); SerialDEBUG.print(map_x); SerialDEBUG.print("/"); SerialDEBUG.println(map_y);

        torque = utils::change(mg2_speed_temp, speedrange_drive[speed_index], speedrange_drive[speed_index + 1], map_x, map_y);
        //SerialDEBUG.print("Torque "); SerialDEBUG.print(torque), SerialDEBUG.print(", Throttle "); SerialDEBUG.print(ThrotVal), SerialDEBUG.print(", Speed "); SerialDEBUG.println(mg2_speed);

        // FULL TORQUE!!!
        //torque = (long)torque * 1750 / 3500;
    }

    if (Param::GetInt(Param::dir) == -1) {
        if (mg2_speed_temp < speedrange_reverse[0]) mg2_speed_temp = speedrange_reverse[0]; // force min speed if speed below expected range
        pedal_index = utils::change(ThrotVal, pedal_range[0], pedal_range[5], 0, 5);
        speed_index = utils::change(mg2_speed_temp, speedrange_reverse[0], speedrange_reverse[4], 0, 4);

        if (pedal_index >= 5 && speed_index >= 4) return (pedalmap_drive[10][4]); // pedal and speed maxed out
        if (pedal_index >= 5) return (utils::change(mg2_speed_temp, speedrange_reverse[speed_index], speedrange_reverse[speed_index + 1], pedalmap_reverse[speed_index][5], pedalmap_reverse[speed_index + 1][5])); // pedal maxed out
        if (speed_index >= 4) return (utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[4][pedal_index], pedalmap_reverse[4][pedal_index + 1])); // speed maxed out

        map_x = utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[speed_index][pedal_index], pedalmap_reverse[speed_index][pedal_index + 1]);
        map_y = utils::change(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[speed_index + 1][pedal_index], pedalmap_reverse[speed_index + 1][pedal_index + 1]);

        torque = utils::change(mg2_speed_temp, speedrange_reverse[speed_index], speedrange_reverse[speed_index + 1], map_x, map_y);

        // Scaling already happening in map
        // torque = (long)torque * 1750 / 3500;
    }

    if (Param::GetInt(Param::dir) == 0) torque = 0;//no torque in neutral
    return torque; //return torque
}


void GS450HClass::setTimerState(bool desiredTimerState)
{
   if (desiredTimerState != this->timerIsRunning)
   {
      if (desiredTimerState)
      {
         tim_setup(); //toyota hybrid oil pump pwm timer
         tim2_setup(); //TOYOTA HYBRID INVERTER INTERFACE CLOCK
         this->timerIsRunning=true; //timers are now running
      }
      else
      {
         // These are only used with the Totoa hybrid option.
         timer_disable_counter(TIM2); //TOYOTA HYBRID INVERTER INTERFACE CLOCK
         timer_disable_counter(TIM1); //toyota hybrid oil pump pwm timer
         this->timerIsRunning=false; //timers are now stopped
      }
   }
}

void GS450HClass::setTorqueTarget(int16_t torquePercent)
{
    // this->scaledTorqueTarget = utils::change(torquePercent, 0, 3040, 0, 3500);//map throttle for GS450HClass inverter
    torquePercent = torquePercent;
    this->scaledTorqueTarget = get_torque();
}


// 100 ms code
void GS450HClass::run100msTask(uint8_t Lexus_Gear, uint16_t Lexus_Oil)
{

   Param::SetInt(Param::InvStat, GS450HClass::statusFB()); //update inverter status on web interface

   if (Lexus_Gear == 1)
   {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Clear();
      DigIo::SL2_out.Clear();

      Param::SetInt(Param::GearFB,HIGH_Gear);// set high gear
   }

   if (Lexus_Gear == 0)
   {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Clear();
      DigIo::SL2_out.Clear();

      Param::SetInt(Param::GearFB,LOW_Gear);// set low gear
   }
   setTimerState(true);

   uint16_t Lexus_Oil2 = utils::change(Lexus_Oil, 10, 80, 1875, 425); //map oil pump pwm to timer
   timer_set_oc_value(TIM1, TIM_OC1, Lexus_Oil2);//duty. 1000 = 52% , 500 = 76% , 1500=28%

   Param::SetInt(Param::Gear1,DigIo::gear1_in.Get());//update web interface with status of gearbox PB feedbacks for diag purposes.
   Param::SetInt(Param::Gear2,DigIo::gear2_in.Get());
   Param::SetInt(Param::Gear3,DigIo::gear3_in.Get());

   Param::SetInt(Param::tmphs,GS450HClass::temp_inv_water);//send GS450H inverter temp to web interface

   static s32fp mTemps[2];
   static s32fp tmpm;

    int tmpmg1 = AnaIn::MG1_Temp.Get();//in the gs450h case we must read the analog temp values from sensors in the gearbox
    int tmpmg2 = AnaIn::MG2_Temp.Get();
    Param::SetInt(Param::tmpmg1,tmpmg1);
    Param::SetInt(Param::tmpmg2,tmpmg2);
    float temp_1 = TempMeas::readLexusThermistor(tmpmg1);
    Param::SetFlt(Param::tmpmg1fp,FP_FROMFLT(temp_1));
    float temp_2 = TempMeas::readLexusThermistor(tmpmg2);
    Param::SetFlt(Param::tmpmg2fp,FP_FROMFLT(temp_2));

    mTemps[0] = FP_FROMFLT(temp_1);
    mTemps[1] = FP_FROMFLT(temp_2);

   tmpm = MAX(mTemps[0], mTemps[1]);//which ever is the hottest gets displayed
   Param::SetFixed(Param::tmpm,tmpm);
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Usart 2 DMA Transmitt and Receive Section
//////////////////////////////////////////////////////////////////////////

static void dma_write(uint8_t* data, int size)
{
   /*
    * Using channel 7 for USART2_TX
    */

   /* Reset DMA channel*/
   dma_channel_reset(DMA1, DMA_CHANNEL7);

   dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
   dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
   dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
   dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
   dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_MEDIUM);

   //dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);
   //dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);
   dma_enable_channel(DMA1, DMA_CHANNEL7);

   usart_enable_tx_dma(USART2);
}

volatile int transfered = 0;

void dma1_channel7_isr(void)
{


   if ((DMA1_ISR &DMA_ISR_TCIF7) != 0)
   {
      DMA1_IFCR |= DMA_IFCR_CTCIF7;//Interrupt Flag Clear Register

      transfered = 1;
   }

   dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

   usart_disable_tx_dma(USART2);

   dma_disable_channel(DMA1, DMA_CHANNEL7);


}

static void dma_read(uint8_t* data, int size)
{
   /*
    * Using channel 6 for USART2_RX
    */

   /* Reset DMA channel*/
   dma_channel_reset(DMA1, DMA_CHANNEL6);

   dma_set_peripheral_address(DMA1, DMA_CHANNEL6, (uint32_t)&USART2_DR);
   dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t)data);
   dma_set_number_of_data(DMA1, DMA_CHANNEL6, size);
   dma_set_read_from_peripheral(DMA1, DMA_CHANNEL6);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
   dma_set_priority(DMA1, DMA_CHANNEL6, DMA_CCR_PL_LOW);

   //dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);
   // dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
   dma_enable_channel(DMA1, DMA_CHANNEL6);

   usart_enable_rx_dma(USART2);
}

volatile int received = 0;

void dma1_channel6_isr(void)
{
   if ((DMA1_ISR &DMA_ISR_TCIF6) != 0)
   {
      DMA1_IFCR |= DMA_IFCR_CTCIF6;

      received = 1;
   }

   dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);

   usart_disable_rx_dma(USART2);

   dma_disable_channel(DMA1, DMA_CHANNEL6);
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dilbert's code here
//////////////////////////////////////////////////////////////////////////////////////////////////////

void GS450HClass::SetPrius()
{

   if (htm_state<5)
   {
      htm_state = 5;
      inv_status = 0;//must be 0 for prius
   }
}
void GS450HClass::SetGS450H()
{
   if (htm_state>4)
   {
      htm_state = 0;
      inv_status = 1;//must be 1 for gs450h
   }
}



uint8_t GS450HClass::VerifyMTHChecksum(uint16_t len)
{

   uint16_t mth_checksum=0;

   for(int i=0; i<(len-2); i++)
      mth_checksum+=mth_data[i];


   if(mth_checksum==(mth_data[len-2]|(mth_data[len-1]<<8))) return 1;
   else return 0;

}

void GS450HClass::CalcHTMChecksum(uint16_t len)
{

   uint16_t htm_checksum=0;

   for(int i=0; i<(len-2); i++)htm_checksum+=htm_data[i];
   htm_data[len-2]=htm_checksum&0xFF;
   htm_data[len-1]=htm_checksum>>8;

}

void GS450HClass::UpdateHTMState1Ms(int8_t gear)
{

   switch(htm_state)
   {

   case 0:
   {
      dma_read(mth_data,100);//read in mth data via dma. Probably need some kind of check dma complete flag here
      DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
      htm_state++;
   }
   break;

   case 1:
   {
      DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

      if(inv_status==0)
      {
         if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))// if the transfer complete flag is set then send another packet
         {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);//clear the flag.
            dma_write(htm_data,80); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
         }

      }
      else
      {
         dma_write(htm_data_setup,80);   //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);
         if(mth_data[1]!=0)
            inv_status--;
      }
      htm_state++;
      break;

      case 2:
         htm_state++;
      }
      break;

   case 3:
   {
      //
      // dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF);
      if(VerifyMTHChecksum(100)==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0)
      {
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
         statusInv=0;
      }
      else
      {
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
//exchange data and prepare next HTM frame
         dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
         statusInv=1;
         dc_bus_voltage=(((mth_data[82]|mth_data[83]<<8)-5)/2);
         temp_inv_water=(mth_data[42]|mth_data[43]<<8);
         temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
         mg1_speed=mth_data[6]|mth_data[7]<<8;
         mg2_speed=mth_data[31]|mth_data[32]<<8;
      }

      mth_data[98]=0;
      mth_data[99]=0;

      htm_state++;
   }
   break;

   case 4:
   {

      // -3500 (reverse) to 3500 (forward)
      if(gear==0) mg2_torque=0;//Neutral
      if(gear==32) mg2_torque=this->scaledTorqueTarget;//Drive
      if(gear==-32) mg2_torque=this->scaledTorqueTarget;//Reverse

      mg1_torque=((mg2_torque*5)/4);
      if(gear==-32) mg1_torque=0; //no mg1 torque in reverse.
      Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

      //speed feedback
      speedSum=mg2_speed+mg1_speed;
      speedSum/=113;
      uint8_t speedSum2=speedSum;
      htm_data[0]=speedSum2;
      htm_data[75]=(mg1_torque*4) & 0xFF;
      htm_data[76]=((mg1_torque*4)>>8) & 0xFF;

      //mg1
      htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
      htm_data[6]=((mg1_torque*-1)>>8);
      htm_data[11]=htm_data[5];
      htm_data[12]=htm_data[6];

      //mg2
      htm_data[26]=(mg2_torque) & 0xFF; //positive is forward
      htm_data[27]=((mg2_torque)>>8) & 0xFF;
      htm_data[32]=htm_data[26];
      htm_data[33]=htm_data[27];

      htm_data[63]=(-5000)&0xFF;  // regen ability of battery
      htm_data[64]=((-5000)>>8);

      htm_data[65]=(27500)&0xFF;  // discharge ability of battery
      htm_data[66]=((27500)>>8);

      //checksum
      htm_checksum=0;
      for(int i=0; i<78; i++)htm_checksum+=htm_data[i];
      htm_data[78]=htm_checksum&0xFF;
      htm_data[79]=htm_checksum>>8;

      if(counter>100)
      {
//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
         counter = 0;
      }
      else
      {
         counter++;
      }

      htm_state=0;
   }
   break;

   /***** Demo code for Gen3 Prius/Auris direct communications! */
   case 5:
   {
      dma_read(mth_data,120);//read in mth data via dma. Probably need some kind of check dma complete flag here
      DigIo::req_out.Clear(); //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
      htm_state++;
   }
   break;

   case 6:
   {
      DigIo::req_out.Set();  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

      if(inv_status>5)
      {
         if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))// if the transfer complete flag is set then send another packet
         {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF);//clear the flag.
            dma_write(htm_data,100); //HAL_UART_Transmit_IT(&huart2, htm_data, 80);
         }

      }
      else
      {
         dma_write(&htm_data_init[ inv_status ][0],100); //HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

         inv_status++;
         if(inv_status==6)
         {
//memcpy(htm_data, &htm_data_init[ inv_status ][0], 100);
         }
      }
      htm_state++;
      break;

      case 7:
         htm_state++;
      }
      break;

   case 8:
   {
      //
      // dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF);
      if(VerifyMTHChecksum(120)==0 || dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF)==0)
      {

         statusInv=0;
      }
      else
      {

         //exchange data and prepare next HTM frame
         dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
         statusInv=1;
         dc_bus_voltage=(((mth_data[100]|mth_data[101]<<8)-5)/2);
         temp_inv_water=(mth_data[42]|mth_data[43]<<8);
         temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
         mg1_speed=mth_data[6]|mth_data[7]<<8;
         mg2_speed=mth_data[38]|mth_data[39]<<8;
      }

      mth_data[98]=0;
      mth_data[99]=0;

      htm_state++;
   }
   break;

   case 9:
   {

      // -3500 (reverse) to 3500 (forward)
      if(gear==0) mg2_torque=0;//Neutral
      if(gear==32) mg2_torque=this->scaledTorqueTarget;//Drive
      if(gear==-32) mg2_torque=this->scaledTorqueTarget*-1;//Reverse

      mg1_torque=((mg2_torque*5)/4);
      if(gear==-32) mg1_torque=0; //no mg1 torque in reverse.
      Param::SetInt(Param::torque,mg2_torque);//post processed final torue value sent to inv to web interface

      //speed feedback
      speedSum=mg2_speed+mg1_speed;
      speedSum/=113;
      //Possibly not needed
      //uint8_t speedSum2=speedSum;
      //htm_data[0]=speedSum2;

      //these bytes are used, and seem to be MG1 for startup, but can't work out the relatino to the
      //bytes earlier in the stream, possibly the byte order has been flipped on these 2 bytes
      //could be a software bug ?
      htm_data[76]=(mg1_torque*4) & 0xFF;
      htm_data[75]=((mg1_torque*4)>>8) & 0xFF;

      //mg1
      htm_data[5]=(mg1_torque)&0xFF;  //negative is forward
      htm_data[6]=((mg1_torque)>>8);
      htm_data[11]=htm_data[5];
      htm_data[12]=htm_data[6];

      //mg2 the MG2 values are now beside each other!
      htm_data[30]=(mg2_torque) & 0xFF; //positive is forward
      htm_data[31]=((mg2_torque)>>8) & 0xFF;

      if(gear==32)   //forward direction these bytes should match
      {
         htm_data[26]=htm_data[30];
         htm_data[27]=htm_data[31];
         htm_data[28]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[29]=((mg2_torque/2)>>8) & 0xFF;
      }

      if(gear==-32)   //reverse direction these bytes should match
      {
         htm_data[28]=htm_data[30];
         htm_data[29]=htm_data[31];
         htm_data[26]=(mg2_torque/2) & 0xFF; //positive is forward
         htm_data[27]=((mg2_torque/2)>>8) & 0xFF;
      }

      //This data has moved!

      htm_data[85]=(-5000)&0xFF;  // regen ability of battery
      htm_data[86]=((-5000)>>8);

      htm_data[87]=(-10000)&0xFF;  // discharge ability of battery
      htm_data[88]=((-10000)>>8);

      //checksum
      if(++frame_count & 0x01)
      {
         htm_data[94]++;
      }

      CalcHTMChecksum(100);

      htm_state=5;
   }
   break;

   }
}


bool GS450HClass::statusFB()
{
   return statusInv;
}
//////////////////////////////////////////////////////////////
