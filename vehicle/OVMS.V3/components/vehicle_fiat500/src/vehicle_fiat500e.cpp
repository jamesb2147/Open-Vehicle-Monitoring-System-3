/*
;    Project:       Open Vehicle Monitor System
;    Date:          5th July 2018
;
;    Changes:
;    1.0  Initial release
;
;
;
;    (C) 2021       Guenther Huck
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2018  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
;
; Can bus: CAN1 C-CAN(500kbps); CAN2 B-CAN(50kbps)
; 
; MsgId        bus     Msg                  Signal    variabl                               factor/offset  status  
; 0xC10A040    can1   MSG09_BPCM                   ms_v_bat_soc                SOC                          ok
; 0xC10A040    can1   MSG09_BPCM                   ms_v_bat_range_est          EStRANGE                     ok
; 0xC10A040    can1   MSG09_BPCM                   ms_v_bat_energy_used_total  EStRANGE                    test
; 0xA18A000    can1   STATUS_B_CAN  19/1           ms_v_door_fl                DriveDoorSts                 ok
; 0xA18A000    can1   STATUS_B_CAN  53/1           ms_v_env_handbrake          HandBrakeSts                 ok
; 0xA18A000    can1   STATUS_B_CAN  24/8           ms_v_env_temp               ExternalTemp      5  -40  wrong value
; 0xA18A040    can1   STATUS_C_EVCU 28/3           ms_v_charge_state           ChargingSystemSts            test
;                                                  ms_v_charge_inprogress                                   test
;                                                  ms_v_door_chargeport                                     test
; 0xA28A000    can1   VEHICLE_SPEED_ODOMETER 52/20 ms_v_pos_odometer           TotalOdometer             wrong value     
; ***********************************************************************************************
;
;
;
; ***********************************************************************************************
; 0xC41401f    can2   TCU_REQ       0x20           CommandStartCharge          TCU_ChargeNow_Req
; 0xC41401f    can2   TCU_REQ       0x40           CommandStopCharge           TCU_ChargeNow_Req
;
; 0xC41401f    can2   TCU_REQ       0x8            CommandLock                 TCU_DoorLockCmd
; 0xC41401f    can2   TCU_REQ       0x10           CommandUnLock               TCU_DoorLockCmd
;                                                  ms_v_env_locked
; 0xC41401f    can2   TCU_REQ       0x2                                        TCU_HornLights_Req
; 0xC41401f    can2   TCU_REQ       0x0                                        TCU_HornLights_Req
;
; 0xE194031    can2   TCU_REQ       0x10      5/3  CommandActivateValet        TCU_PreCondNow_Req      PreConditionNow         
; 0xE194031    can2   TCU_REQ       0x20      5/3  CommandDeactivateValet      TCU_PreCondNow_Req      StopPreconditionNow
;  
;
*/

#include "ovms_log.h"
static const char *TAG = "v-fiat500e";

#include <stdio.h>
#include "vehicle_fiat500e.h"
#include "metrics_standard.h"


// Seconds of CAN silence after which the vehicle is considered asleep.
// Matches the value the Bolt module has used in the field (VA_CANDATA_TIMEOUT).
#define FT_CANDATA_TIMEOUT 10

OvmsVehicleFiat500e::OvmsVehicleFiat500e()
  {
  ESP_LOGI(TAG, "Start Fiat 500e vehicle module");

  // Init metrics:
  ft_v_acelec_pwr  = MyMetrics.InitFloat("xse.v.b.acelec.pwr", SM_STALE_MID, 0, Watts);
  ft_v_htrelec_pwr  = MyMetrics.InitFloat("xse.v.b.htrelec.pwr", SM_STALE_MID, 0, Watts);

  // Start at zero rather than at the timeout, so a module booted next to a
  // sleeping car stays quiet instead of announcing a sleep transition it never
  // actually observed.
  m_candata_timer = 0;

  RegisterCanBus(1,CAN_MODE_ACTIVE,CAN_SPEED_500KBPS);
  RegisterCanBus(2,CAN_MODE_ACTIVE,CAN_SPEED_50KBPS);
  }

OvmsVehicleFiat500e::~OvmsVehicleFiat500e()
  {
  ESP_LOGI(TAG, "Stop Fiat 500e vehicle module");
  MyMetrics.DeregisterMetric(ft_v_acelec_pwr);
  MyMetrics.DeregisterMetric(ft_v_htrelec_pwr);
  }

void OvmsVehicleFiat500e::CanActivity()
  {
  if (m_candata_timer == 0)
    ESP_LOGI(TAG, "Car has woken (CAN bus activity)");
  m_candata_timer = FT_CANDATA_TIMEOUT;
  // Idempotent: the metric only signals an event when the value actually
  // changes, so calling this per frame costs a comparison and nothing more.
  StandardMetrics.ms_v_env_awake->SetValue(true);
  }

void OvmsVehicleFiat500e::Ticker1(uint32_t ticker)
  {
  // Without this the module never learns the car has slept: ms_v_env_awake was
  // never set either way, so vehicle.asleep never fired and the framework's
  // auto-poweroff never engaged, leaving the module drawing from the 12V
  // battery indefinitely on a parked car.
  //
  // ms_v_env_on is deliberately left alone. Nothing in this module ever sets it
  // true (there is no ignition-state decode yet), so driving only its false edge
  // would emit a spurious vehicle.off on the first timeout and nothing after.
  if (m_candata_timer > 0 && --m_candata_timer == 0)
    {
    ESP_LOGI(TAG, "Car has gone to sleep (CAN bus timeout)");
    StandardMetrics.ms_v_env_awake->SetValue(false);
    }
  }

class OvmsVehicleFiat500eInit
  {
  public: OvmsVehicleFiat500eInit();
  } 
  OvmsVehicleFiat500eInit  __attribute__ ((init_priority (9000)));
  OvmsVehicleFiat500eInit::OvmsVehicleFiat500eInit() 
  {
    ESP_LOGI(TAG, "Registering Vehicle: FIAT 500e (9000)");
    MyVehicleFactory.RegisterVehicle<OvmsVehicleFiat500e>("FT5E","Fiat 500e");  
      //StandardMetrics.ms_v_door_chargeport->SetValue(false); // false (for test only)
      //StandardMetrics.ms_v_env_locked->SetValue(true); // True for Test only
      //StandardMetrics.ms_v_pos_odometer->SetValue(0);
      //StandardMetrics.ms_v_env_temp->SetValue(0);
      //StandardMetrics.ms_v_env_cabintemp->SetValue(0);
      //StandardMetrics.ms_v_bat_soc->SetValue(0);
      //StandardMetrics.ms_v_bat_range_est->SetValue(0);
      //StandardMetrics.ms_v_bat_temp->SetValue(0);
      //StandardMetrics.ms_v_bat_voltage->SetValue(0);
  } 

//*****************************************************************************
//****************** Incoming Frames Can-Bus C (500kbps) **********************
//*****************************************************************************

void OvmsVehicleFiat500e::IncomingFrameCan1(CAN_frame_t* p_frame) {

  CanActivity();

  
  uint8_t *d = p_frame->data.u8;
  
  switch (p_frame->MsgID) { 
    case 0xC10A040: // MSG31B_EVCU
    {
      StandardMetrics.ms_v_bat_range_est->SetValue(d[0]);
      //Est_Range (estimated range)
      //d[0] 11111111 
      float soc = d[1] >> 1;
      StandardMetrics.ms_v_bat_soc->SetValue(soc);
      //SOC_Display (state of charge)
      //d[0] 11111110
      float pow = ((uint16_t) d[2] << 5) | (d[3] >>3);
      StandardMetrics.ms_v_bat_energy_used->SetValue((pow/100)-24);
      //möglicherweise aktueller ENergieverbrauch in kw?
      //HVTotalEnergy (Energieverbrauch X*0,01-24) in kwh?
      //d[2] 11111111
      //d[3] 11111000
      break;
    }   
    case 0x820A040: //MSG29_EVCU
    {
      // J1772_S2_Close is the vehicle-side S2 switch. Closed means the vehicle
      // has completed the charge circuit (J1772 state C), i.e. energy transfer
      // is under way, which matches ms_v_charge_inprogress ("True = currently
      // charging"). This frame is the sole owner of that metric.
      //J1772_S2_Close //0x0 open //0x1 closed
      //d[1] 00010000
      StandardMetrics.ms_v_charge_inprogress->SetValue(d[1]&0x10);
      break;
    }
    case 0x640A046: //MSG06_BPCM
    {
      // RdyForChrg is a BPCM readiness/permission flag, not an indication that
      // charging is under way. It previously also wrote ms_v_charge_inprogress,
      // so this frame and 0x820A040 above contended for the same boolean and
      // flipped it on nearly every frame whenever the two disagreed -- for
      // example while plugged in but not yet drawing current.
      //
      // Each flip signals vehicle.charge.start / vehicle.charge.stop, and every
      // event heap-allocates and queues its name. A dropped non-ticker event is
      // a deliberate abort() (ovms_events.cpp CheckQueueOverflow), so the
      // contention was a reboot mechanism, not just noisy reporting.
      //
      // The correct destination for RdyForChrg is not established from the bus
      // documentation available, so it is left undecoded rather than guessed at.
      //RdyForChrg (ready for charge)
      //d[5] 00000100
      break;
    }
    case 0x400A042: //EM_02
    {
      //StandardMetrics. ms_v_mot_temp->SetValue(d[0]*0.5-25);
      StandardMetrics. ms_v_mot_temp->SetValue(((float)d[0])-50);
      //TempCurrMot (temperature of e-machine)
      //d[0] 11111111
      StandardMetrics. ms_v_gen_temp->SetValue(((float)d[1])-50);
      //TempCurrRotor (temperature of rotor)
      //d[1] 11111111
       StandardMetrics. ms_v_inv_temp->SetValue(((float)d[2])-50);
         //StandardMetrics. ms_v_inv_temp->SetValue(d[2]*0.5-25);
      //TempCurrPWR (temperature of inverter)
      //d[2] 11111111
      break;
    }
    case 0xC08A040: //MSG30_EVCU
    {
      ft_v_acelec_pwr->SetValue(d[0]*4);
      //ACElecPwr (Aircondition electric power)
      //d[0] 11111111
      ft_v_htrelec_pwr->SetValue(d[4]*4);
      //HtrElecPwr (Heater electric power)
      //d[1] 11111111
      break;
    }
    case 0xC50A049: //MSG36_OBCM
    {
      unsigned int cvolt = ((unsigned int)d[1]<<8)
                          + (unsigned int)d[2];
      StandardMetrics.ms_v_charge_voltage->SetValue((float)cvolt/10);

      unsigned int ccurr = ((unsigned int)d[3]<<8)
                          | (unsigned int)d[4];
      StandardMetrics.ms_v_charge_current->SetValue((float)ccurr/5-50);

      unsigned int bvolt = ((unsigned int)d[5]<<8)
                          | (unsigned int)d[6];
      StandardMetrics.ms_v_bat_voltage->SetValue((float)bvolt/10);
      
      StandardMetrics.ms_v_charge_temp->SetValue(d[0]);
     
      break;
    }
    case 0x840A046: //MSG08_BPCM 
    {
      float btemp = (((d[2] & 0x7f) << 1) | (d[2] & 0x80));
      //StandardMetrics.ms_v_bat_temp->SetValue(btemp*0.5-40);
      StandardMetrics.ms_v_bat_temp->SetValue(btemp-40);
      //StandardMetrics.ms_v_bat_temp->SetValue(((d[2]<<1)|(d[3]&0x80))-40);
      //ModTempAvg (Modultemperatur)
      //d[2] 011111111 
      //d[3]           100000000
      break;
    }
    /*
    case 0xA18A040: // STATUS_C_EVCU
    {
      if (((d[3]&0x70)>>4) == 0x0) {
        //ChargingSystemSts (0x0 Not Charging)
        //d[3] 01110000
        StandardMetrics.ms_v_charge_inprogress->SetValue(false); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(false); // False
        StandardMetrics.ms_v_charge_state->SetValue("topoff");
        break;
      }
      if (((d[3]&0x70)>>4) == 0x1) {
        //ChargingSystemSts (0x1 Charging)
        //d[3] 01110000
	      StandardMetrics.ms_v_charge_inprogress->SetValue(true); // True
        StandardMetrics.ms_v_door_chargeport->SetValue(true); // True
        StandardMetrics.ms_v_charge_state->SetValue("charging");
        break;
      }
      if (((d[3]&0x70)>>4) == 0x2) {
        //ChargingSystemSts (0x2 Charge Interrupted)
        //d[3] 01110000
	      StandardMetrics.ms_v_charge_inprogress->SetValue(true); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(true); // True
        StandardMetrics.ms_v_charge_state->SetValue("stopped");
        break;
      }
      if (((d[3]&0x70)>>4) == 0x3) {  //ChargingSystemSts (0x3 Charge Complete)
        //d[3] 01110000
	      StandardMetrics.ms_v_charge_inprogress->SetValue(false); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(false); // True
        StandardMetrics.ms_v_charge_state->SetValue("done");
        break;
      }
      break;
    }
    default:
    //ESP_LOGD(TAG, "IFC %03x 8 %02x %02x %02x %02x %02x %02x %02x %02x",
    //p_frame->MsgID, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
    break;
    }
    */
  }
}
//*****************************************************************************
//****************** Incoming Frames Can-Bus B (50kbps) ***********************
//*****************************************************************************

void OvmsVehicleFiat500e::IncomingFrameCan2(CAN_frame_t* p_frame) {

  CanActivity();

  
  uint8_t *d = p_frame->data.u8;
  
  switch (p_frame->MsgID) {
    /*
    case 0xC414003: // IPC_DISPLAY
    {
      float soc = ((uint8_t)d[0]>>1);
      //SOC_Display (state of charge)
      //d[0] 11111110
      
      float estrange = ((uint8_t)d[1]>>1);
      //Est_Range (estimated range)
      //d[1] 11111110 
      
      float pow = ((uint16_t) d[6] << 4) | (d[3]&0xF0);
      //möglicherweise aktueller ENergieverbrauch in kw?
      //HVTotalEnergy (Energieverbrauch X*0,01-24) in kwh?
      //d[4] 00001111
      //d[3] 11100000

      StandardMetrics.ms_v_bat_soc->SetValue(soc); 
      StandardMetrics.ms_v_bat_range_est->SetValue(estrange);
      StandardMetrics.ms_v_bat_energy_used->SetValue((pow/100)-24); 
      break;
    }
    */
    case 0x6214000: // STATUS_BCM
    {
      StandardMetrics.ms_v_env_handbrake->SetValue(d[0]&0x20);    
      //HandBrakeSts (0x0 off, 0x1 on)
      StandardMetrics.ms_v_door_fl->SetValue(d[1]&0x4);
      //DriverDoorSts (0x0 closed, 0x1 open)
      StandardMetrics.ms_v_door_fr->SetValue(d[1]&0x8);
      //PsngrDoorSts (0x0 closed, 0x1 open)
      StandardMetrics.ms_v_door_trunk->SetValue(d[1]&0x40);
      //RHatchSts (0x0 closed, 0x1 open)
      //*********** additional parameter *************
      //StandardMetrics.ms_v_?????->SetValue(d[2]&0x4);
      //WarningKeyInSts (0x0 not active, 0x1 active- This signal activates a 
      //buzzer when the driver door is open and the key is in the starting system)
      //***********************************************
      //StandardMetrics.ms_v_?????->SetValue(d[2]&0x8);
      //RechargeSts (0x0 charging, 0x1 not charging)
      //***********************************************
      //StandardMetrics.ms_v_?????->SetValue(d[0]&0x8);
      //InternalLightSts (0x0 not active, 0x1 active)
      //***********************************************
      //StandardMetrics.ms_v_?????->SetValue(d[0]&0x64);
      //BrakeFluidLevelSts (0x0 ok, 0x1 low level)
      break;
    }
    case 0x631400A: // STATUS_ECC2
    {
      unsigned int evapset = ((d[1]& 0x3f) << 2)
                            +((d[2]& 0xf8) >> 4);
      //00111111 11110000
      StandardMetrics.ms_v_env_cabinsetpoint->SetValue(((float)evapset));
      //StandardMetrics.ms_v_env_cabintemp->SetValue(d[3]*0.5-40);
      //StandardMetrics.ms_v_env_cabintemp->SetValue(((float)(d[3])-51));
      //Cabin Temp Set/Evap Temp Target
      //***********
      // PreCondCabinSts lives in d[1] bits 6-7: 0 = off, 1 = active,
      // 2 = set point reached. Every non-off state means the cabin is being
      // conditioned, so they all map to active.
      //
      // "Set point reached" (0x80) was previously unhandled and fell through to
      // default: -> off. A climate system holding temperature cycles through
      // that state continuously, so the metric toggled once per thermostat
      // cycle. See tests/test_transitions.cpp.
      //
      // This drives ms_v_env_hvac, not ms_v_env_valet. Preconditioning is
      // climate control; reporting it as valet mode made the framework send a
      // "Valet mode enabled" notification on every precondition and arm the
      // hood/trunk intrusion alerts (vehicle.cpp NotifyValetHood /
      // NotifyValetTrunk) for as long as it ran.
      //d[1] 11000000
      switch (d[1] & 0xC0) {
        case 0x00:      // off
          StandardMetrics.ms_v_env_hvac->SetValue(false);
          break;
        case 0x40:      // actively conditioning
        case 0x80:      // set point reached, still conditioning
        case 0xC0:      // not described in the available bus documentation;
                        // treated as active, as it was before this change
          StandardMetrics.ms_v_env_hvac->SetValue(true);
          break;
      }
      break;
    }
    case 0xA194040: // STATUS_B_EVCU
    {
      if ((d[3]&0x30) == 0) {
        //ChargingSystemSts (0x0 Not Charging)
        //d[3] 01110000
        //StandardMetrics.ms_v_charge_inprogress->SetValue(false); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(false); // False
        StandardMetrics.ms_v_charge_state->SetValue("topoff");
        break;
      }
      if ((d[3]&0x30) == 0x1) {
        //ChargingSystemSts (0x1 Charging)
        //d[3] 01110000
	      //StandardMetrics.ms_v_charge_inprogress->SetValue(true); // True
        StandardMetrics.ms_v_door_chargeport->SetValue(true); // True
        StandardMetrics.ms_v_charge_state->SetValue("charging");
        break;
      }
      if ((d[3]&0x30) == 0x2) {
        //ChargingSystemSts (0x2 Charge Interrupted)
        //d[3] 01110000
	      //StandardMetrics.ms_v_charge_inprogress->SetValue(true); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(true); // True
        StandardMetrics.ms_v_charge_state->SetValue("stopped");
        break;
      }
      if ((d[3]&0x30) == 0x3) {
        //ChargingSystemSts (0x3 Charge Complete)
        //d[3] 01110000
	      //StandardMetrics.ms_v_charge_inprogress->SetValue(false); // False
        StandardMetrics.ms_v_door_chargeport->SetValue(false); // True
        StandardMetrics.ms_v_charge_state->SetValue("done");
        break;
      }
      break;
    }
    case 0x6414000: // STATUS_BCM4
    {
      StandardMetrics.ms_v_env_locked->SetValue(d[3]&0x40);
      //DriverDoorLogicSts (0x0 unlocked, 0x1 locked)
      break;
    }
    case 0x63D4000: // ENVIRONMENTAL_CONDITIONS
    {
      // Both signals live in this frame. The temperature branch used to end in
      // `break`, which left the voltage read below reachable only when the
      // ambient temperature happened to be exactly zero.
      if (d[0] != 0) {
        StandardMetrics.ms_v_env_temp->SetValue((d[0]*0.5)-40);
        //ExternalTemperature (Aussentemperatur)
        //d[0] 11111111
      }

      // BatteryVoltageLevel maxes out at (0x7f * 0.16) = 20.3V, so this is the
      // 12V accessory battery, not the HV pack. It previously wrote
      // ms_v_bat_voltage -- which is the HV pack voltage and is also written
      // from OBCM frame 0xC50A049 with values around 400V, so the two sources
      // fought and the metric oscillated between roughly 400 and 13.
      //
      // Routing it to ms_v_bat_12v_voltage also gives the framework's 12V
      // monitoring something to read (vehicle.cpp 12v.shutdown / 12v.alert),
      // which previously had no source on this vehicle.
      float vbat = (d[1]&0x7F);
      StandardMetrics.ms_v_bat_12v_voltage->SetValue(vbat*0.16);
      //BatteryVoltageLevel (X*0.16 in Volt)
      //d[1] 01111111
      break;
    }
  /*
    case 0x6814000: // ENVIRONMENTAL_CONDITIONS
     {
     //Motortemperatur ? CoreTemp
      //StandardMetrics.ms_v_bat_temp->SetValue(d[6]*0.1);
      //Sensor
      //StandardMetrics.ms_v_bat_temp->SetValue(d[5]-40);
      //break;
  */
    case 0xC414000: // HUMIDITY_000
    {
    //HumSenAirTemp / BEV NEW. Humidity element temperature
    // The mask needs parenthesising: >> binds tighter than &, so
    // `d[4] & 0x80 >> 7` was evaluated as `d[4] & (0x80 >> 7)`, i.e. `d[4] & 1`.
    // That read the bottom bit of d[4] instead of the intended top bit, so the
    // low bit of the temperature was always wrong.
    float itemp = ((uint16_t) d[3] << 1) | ((d[4] & 0x80) >> 7);
    StandardMetrics.ms_v_env_cabintemp->SetValue(itemp*0.5-40);
    //01111111 10000000
    break;
    }
     case 0xC014003: //TRIP_A_B
    {
     	 float vodo = ((d[1] & 0x0f) << 16) | (d[2] << 8) | d[3];
       StandardMetrics.ms_v_pos_odometer->SetValue(vodo);
      //TotalOdometer (km)
      // 00001111 11111111 11111111
      break;
    }
/* 
    case 0x631400A: //STATUS_ECC2
    {
   	 if ((d[1]&0xC0) == 0x0) {
	      StandardMetrics.ms_v_env_valet->SetValue(false);
        //ChargingSystemSts (0x0 Off)
        //d[1] 11000000
        break;
        }
      if ((d[1]&0xC0) == 0x1) {
	      StandardMetrics.ms_v_env_valet->SetValue(true);
        //ChargingSystemSts (0x1 currently preconditioning)
        //d[1] 11000000
        break;
        }
      if ((d[1]&0xC0) == 0x2) {
	      StandardMetrics.ms_v_env_valet->SetValue(false);
        //ChargingSystemSts (0x2 Set point reached/Maintaining)
        //d[1] 11000000
        break;
      }
      break;
    }
    default:
    //ESP_LOGD(TAG, "IFC %03x 8 %02x %02x %02x %02x %02x %02x %02x %02x",
    //p_frame->MsgID, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  break;
    }  
  */
  }  
}

//*****************************************************************************
//************************ Commands (send on Can-bus) *************************
//*****************************************************************************

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandWakeup() {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext; 
  frame.MsgID = 0xE094000; // NWM_BCM
  frame.FIR.B.DLC = 6;
  frame.data.u8[0] = 0x0;
  frame.data.u8[1] = 0x1;  // SystemCommand // 0x1 wakeup / 0x2 stay active
  frame.data.u8[2] = 0x0;
  frame.data.u8[3] = 0x0;
  frame.data.u8[4] = 0x0;
  frame.data.u8[5] = 0x0;
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandStartCharge() {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext;
  frame.MsgID = 0xC41401f;  // TCU_REQ
  frame.FIR.B.DLC = 1;
  frame.data.u8[0] = 0x20;  // TCU_ChargeNow_Req // start charge now
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandStopCharge() {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext;
  frame.MsgID = 0xC41401f;  // TCU_REQ
  frame.FIR.B.DLC = 1;
  frame.data.u8[0] = 0x40;  // TCU_ChargeNow_Req // stop charge now
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandLock(const char* pin) {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext;
  frame.MsgID = 0xC41401f;  // TCU_REQ
  frame.FIR.B.DLC = 1;
  frame.data.u8[0] = 0x8;  // TCU_DoorLockCmd // lock
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  //StandardMetrics.ms_v_env_locked->SetValue(true);
  return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandUnlock(const char* pin) {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext;
  frame.MsgID = 0xC41401f;  // TCU_REQ 
  frame.FIR.B.DLC = 1;
  frame.data.u8[0] = 0x10;  // TCU_DoorLockCmd // unlock
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  //StandardMetrics.ms_v_env_locked->SetValue(false);
  return Success; 
}

// Cabin preconditioning. This is server v2 command 26 (Remote Climate Control),
// and it is also where the framework's own scheduled-preconditioning support
// looks: vehicle.cpp CheckPreconditionSchedule() calls CommandClimateControl()
// and gates on ms_v_env_hvac, so that feature could never drive this car while
// preconditioning lived on the valet commands.
OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandClimateControl(bool enable) {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext;
  frame.MsgID = 0xE194031; // TCU_PRECOND_NOW
  frame.FIR.B.DLC = 2;
  frame.data.u8[0] = enable ? 0x20 : 0x40;  // TCU_PreCondNow_Req // start / stop
  frame.data.u8[1] = 0x64;

  if (enable) {
    // Wake the BCM first: it will not accept a cold precondition request. The
    // stop path deliberately omits this, since a BCM that is currently
    // preconditioning is necessarily already awake. This asymmetry is carried
    // over unchanged from the original valet-mode implementation.
    CAN_frame_t frame_wu = {};
    frame_wu.FIR.B.FF = CAN_frame_ext;
    frame_wu.MsgID = 0xE094000; // NWM_BCM
    frame_wu.FIR.B.DLC = 6;
    frame_wu.data.u8[0] = 0x0;
    frame_wu.data.u8[1] = 0x1;  // SystemCommand // 0x1 wakeup / 0x2 stay active
    frame_wu.data.u8[2] = 0x0;
    frame_wu.data.u8[3] = 0x0;
    frame_wu.data.u8[4] = 0x0;
    frame_wu.data.u8[5] = 0x0;

    m_can2->Write(&frame_wu);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    m_can2->Write(&frame);
  }
  else {
    m_can2->Write(&frame);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    m_can2->Write(&frame);
    // Optimistic local update so the UI responds without waiting for the next
    // STATUS_ECC2 frame on the 50 kbps B-CAN. Retained from the original.
    StandardMetrics.ms_v_env_hvac->SetValue(false);
  }
  return Success;
}

// Preconditioning was originally exposed through valet mode (server v2 commands
// 21/23) because there was no climate-control override. It has moved to
// CommandClimateControl above, but these remain as permanent aliases so that
// existing app buttons, scripts and home-automation integrations bound to valet
// mode keep working. They are supported, not deprecated, and so deliberately do
// not log a warning.
//
// The pin argument is ignored, as it always was here. The base class would call
// PinCheck(), which fails closed when no PIN is configured -- adding that now
// would break exactly the users these aliases exist to protect. See
// docs/index.rst.
//
// This module still does not implement real valet mode.
OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandActivateValet(const char* /*pin*/) {
  return CommandClimateControl(true);
}

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandDeactivateValet(const char* /*pin*/) {
  return CommandClimateControl(false);
}

/*
//Schalte precondition ein / aus

OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandHomelink(int button, int durationms) {
  if (button == 0) {  
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext; 
  frame.MsgID = 0xE194031; // TCU_PRECOND_NOW
  frame.FIR.B.DLC = 2;
  frame.data.u8[0] = 0x20;  // TCU_PreCondNow_Req // PreCondition Start
  frame.data.u8[1] = 0x64;  
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  return Success;
  }
  
  if (button == 1) {
  CAN_frame_t frame = {};
  frame.FIR.B.FF = CAN_frame_ext; 
  frame.MsgID = 0xE194031; // TCU_PRECOND_NOW
  frame.FIR.B.DLC = 2;
  frame.data.u8[0] = 0x40;  // TCU_PreCondNow_Req // PreCondition Stop
  frame.data.u8[1] = 0x64;  
  m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  StandardMetrics.ms_v_env_valet->SetValue(false);
  return Success;
  }

  if (button == 2) {
    CAN_frame_t frame = {};
    frame.FIR.B.FF = CAN_frame_ext;
    frame.MsgID = 0x18DA42f1;  // DIAGNOSTIC_REQUEST_BPCM (ODO)
    frame.FIR.B.DLC = 8;
    frame.data.u8[0] = 0x03;
    frame.data.u8[1] = 0x22;
    frame.data.u8[2] = 0x20;
    frame.data.u8[3] = 0x01;
    frame.data.u8[4] = 0x00;
    frame.data.u8[5] = 0x00;
    frame.data.u8[6] = 0x00;
    frame.data.u8[7] = 0x00;
    m_can1->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can1->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can1->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
    return Success;
  }
return NotImplemented;
}
*/
//Schalte Alarmton ein / aus
OvmsVehicle::vehicle_command_t OvmsVehicleFiat500e::CommandHomelink(int button, int durationms)
{
  //Test of Homelink Alarm!!!
  if (button == 0) {
    CAN_frame_t frame = {};
    frame.FIR.B.FF = CAN_frame_ext;
    frame.MsgID = 0xC41401f;  // TCU_REQ
    frame.FIR.B.DLC = 1;
    frame.data.u8[0] = 0x2;  // HornLightsReq // Enable Light and Horn Flashing
    m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
    return Success; 
  }
  
  if (button == 1) {
    CAN_frame_t frame = {};
    frame.FIR.B.FF = CAN_frame_ext;
    frame.MsgID = 0xC41401f;  // TCU_REQ
    frame.FIR.B.DLC = 1;
    frame.data.u8[0] = 0x0;  // HornLightsReq // Enable Light and Horn Flashing
    m_can2->Write(&frame);
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  m_can2->Write(&frame);
  vTaskDelay(50 / portTICK_PERIOD_MS);
    return Success;
  }
  if (button == 2) {
    CAN_frame_t frame = {};
    frame.FIR.B.FF = CAN_frame_ext;
    frame.MsgID = 0x18DA42f1;  // DIAGNOSTIC_REQUEST_BPCM (ODO)
    frame.FIR.B.DLC = 8;
    frame.data.u8[0] = 0x03;
    frame.data.u8[1] = 0x22;
    frame.data.u8[2] = 0x20;
    frame.data.u8[3] = 0x01;
    frame.data.u8[4] = 0x00;
    frame.data.u8[5] = 0x00;
    frame.data.u8[6] = 0x00;
    frame.data.u8[7] = 0x00;
    
    CAN_frame_t frame_1 = {};
    frame_1.FIR.B.FF = CAN_frame_ext;
    frame_1.MsgID = 0x18DA47f1;  // DIAGNOSTIC_REQUEST_BPCM (ODO)
    frame_1.FIR.B.DLC = 8;
    frame_1.data.u8[0] = 0x03;
    frame_1.data.u8[1] = 0x22;
    frame_1.data.u8[2] = 0x20;
    frame_1.data.u8[3] = 0x01;
    frame_1.data.u8[4] = 0x00;
    frame_1.data.u8[5] = 0x00;
    frame_1.data.u8[6] = 0x00;
    frame_1.data.u8[7] = 0x00;
    m_can1->Write(&frame);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    m_can1->Write(&frame_1);
  
  //vTaskDelay(50 / portTICK_PERIOD_MS);
  //m_can1->Write(&frame);
  //vTaskDelay(50 / portTICK_PERIOD_MS);
  //m_can1->Write(&frame);
  return Success;
  }
  return NotImplemented;
}

