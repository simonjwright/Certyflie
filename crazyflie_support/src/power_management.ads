------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2016, AdaCore                     --
--          Copyright (C) 2020, Simon Wright <simon@pushface.org>           --
--                                                                          --
--  This library is free software;  you can redistribute it and/or modify   --
--  it under terms of the  GNU General Public License  as published by the  --
--  Free Software  Foundation;  either version 3,  or (at your  option) any --
--  later version. This library is distributed in the hope that it will be  --
--  useful, but WITHOUT ANY WARRANTY;  without even the implied warranty of --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                    --
--                                                                          --
--  As a special exception under Section 7 of GPL version 3, you are        --
--  granted additional permissions described in the GCC Runtime Library     --
--  Exception, version 3.1, as published by the Free Software Foundation.   --
--                                                                          --
--  You should have received a copy of the GNU General Public License and   --
--  a copy of the GCC Runtime Library Exception along with this program;    --
--  see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see   --
--  <http://www.gnu.org/licenses/>.                                         --
--                                                                          --
--  As a special exception, if other files instantiate generics from this   --
--  unit, or you link this unit with other files to produce an executable,  --
--  this  unit  does not  by itself cause  the resulting executable to be   --
--  covered by the GNU General Public License. This exception does not      --
--  however invalidate any other reasons why the executable file  might be  --
--  covered by the  GNU Public License.                                     --
------------------------------------------------------------------------------

with Ada.Real_Time;

with LEDS;
with Types;

package Power_Management
is

   --  Types

   --  Type representing the current power state.
   type Power_State is (On_Battery, Charging, Charged, Low_Power, Shut_Down);

   --  Type representing the current charge state.
   type Power_Charge_State is (Charge_100_MA, Charge_500_MA, Charge_MAX);

   --  Types used for Syslink packet translation.
   type Power_Syslink_Info_Repr is (Normal, Flags_Detailed);

   --  Type representing a syslink packet containing power information.
   type Power_Syslink_Info (Repr : Power_Syslink_Info_Repr := Normal) is record
      case Repr is
         when Normal =>
            Flags            : Types.T_Uint8;
            V_Bat_1          : Float;
            Current_Charge_1 : Float;
         when Flags_Detailed =>
            Pgood            : Boolean;
            Charging         : Boolean;
            Unused           : Types.T_Uint6;
            V_Bat_2          : Float;
            Current_Charge_2 : Float;
      end case;
   end record;

   pragma Unchecked_Union (Power_Syslink_Info);
   for Power_Syslink_Info'Size use 72;
   pragma Pack (Power_Syslink_Info);

   --  Procedures and functions

   --  Initialize the power management module.
   procedure Init;

   --  Return True is the Crazyflie is discharging, False when it's charging.
   function Is_Discharging return Boolean;

   --  Get the current battery voltage.
   function Get_Battery_Voltage return Float;

private

   --  Global variables and constants

   --  Current power information received from nrf51
   --  and current power state.
   Current_Power_Info  : Power_Syslink_Info;
   Current_Power_State : Power_State;

   --  Current battery voltage, and its min and max values.
   Battery_Voltage          : Float;
   Battery_Voltage_Min      : Float := 6.0;
   Battery_Voltage_Max      : Float := 0.0;
   Battery_Low_Time_Stamp   : Ada.Real_Time.Time;

   --  LEDs to switch on according power state.
   Charging_LED  : constant LEDS.Crazyflie_LED := LEDS.Blue_L;
   Charged_LED   : constant LEDS.Crazyflie_LED := LEDS.Green_L;
   Low_Power_Led : constant LEDS.Crazyflie_LED := LEDS.Red_L;

   --  Constants used to detect when the battery is low.
   PM_BAT_LOW_VOLTAGE : constant := 3.2;
   PM_BAT_LOW_TIMEOUT : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Seconds (5);

   --  Constants used to know the charge percentage of the battery.
   Bat_671723HS25C : constant array (1 .. 10) of Float :=
                       (
                        3.00, --   00%
                        3.78, --   10%
                        3.83, --   20%
                        3.87, --   30%
                        3.89, --   40%
                        3.92, --   50%
                        3.96, --   60%
                        4.00, --   70%
                        4.04, --   80%
                        4.10  --   90%
                       );

   subtype Charge_State is Natural range 0 .. 9;
   --  0 is completely discharged and 9 is 90% (or more) charged.

   --  Procedures and functions

   --  Set the battery voltage and its min and max values.
   procedure Set_Battery_Voltage (Voltage : Float);

   --  Return a number From 0 To 9 Where 0 is completely Discharged
   --  and 9 is 90% charged.
   function Get_Charge_From_Voltage
     (Voltage : Float) return Charge_State;

   --  Get the power state for the given power information received from
   --  the nrf51.
   function Get_State
     (Power_Info : Power_Syslink_Info) return Power_State;

   --  Switch on/off the power related leds according to power state.
   procedure Set_Power_LEDs (State : Power_State);

end Power_Management;
