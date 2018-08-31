------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2016, AdaCore                     --
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

with Ada.Unchecked_Conversion;

with Log;

package body Power_Management
with Refined_State => (Power_Management_State => (Current_Power_Info,
                                                  Current_Power_State,
                                                  Battery_Voltage,
                                                  Battery_Voltage_Min,
                                                  Battery_Voltage_Max,
                                                  Battery_Low_Time_Stamp))
is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      Current_Power_State := Battery;

      declare
         Dummy : Boolean;
      begin
         Log.Add_Variable (Group    => "pm",
                           Name     => "vbat",
                           Typ      => Log.FLOAT,
                           Variable => Battery_Voltage'Address,
                           Success  => Dummy);
         Log.Add_Variable (Group    => "pm",
                           Name     => "state",
                           Typ      => Log.INT8,
                           Variable => Current_Power_State'Address,
                           Success  => Dummy);
      end;
   end Init;

   -------------------------
   -- Set_Battery_Voltage --
   -------------------------

   procedure Set_Battery_Voltage (Voltage : Float) is
   begin
      Battery_Voltage := Voltage;

      if Battery_Voltage_Max < Voltage then
         Battery_Voltage_Max := Voltage;
      end if;

      if Battery_Voltage_Min > Voltage then
         Battery_Voltage_Min := Voltage;
      end if;
   end Set_Battery_Voltage;

   -------------------------
   -- Get_Battery_Voltage --
   -------------------------

   function Get_Battery_Voltage return Float is
     (Battery_Voltage);

   -----------------------------
   -- Get_Charge_From_Voltage --
   -----------------------------

   function Get_Charge_From_Voltage
     (Voltage : Float) return Integer
   is
   begin
      for Charge in Bat_671723HS25C'Range loop
         if Voltage <= Bat_671723HS25C (Charge) then
            return Charge - 1;
         end if;
      end loop;

      return 9;
   end Get_Charge_From_Voltage;

   --------------------
   -- Syslink_Update --
   --------------------

   procedure Syslink_Update (Sl_Packet : Syslink.Packet) is
      subtype Power_Data is Types.T_Uint8_Array (1 .. 9);
      function Syslink_Data_To_Power_Syslink_Info is
         new Ada.Unchecked_Conversion (Power_Data, Power_Syslink_Info);
   begin
      Current_Power_Info :=
        Syslink_Data_To_Power_Syslink_Info (Sl_Packet.Data (1 .. 9));
      Set_Battery_Voltage (Current_Power_Info.V_Bat_1);
   end Syslink_Update;

   ---------------
   -- Get_State --
   ---------------

   function Get_State
     (Power_Info : Power_Syslink_Info) return Power_State
   is
      State            : Power_State;
      Is_Charging      : Boolean;
      Is_Pgood         : Boolean;
      Charge_Rate      : Integer;
      Battery_Low_Time : Ada.Real_Time.Time_Span;

      use type Ada.Real_Time.Time_Span;
   begin
      Is_Charging      := Power_Info.Charging;
      Is_Pgood         := Power_Info.Pgood;
      Battery_Low_Time := Ada.Real_Time.Clock - Battery_Low_Time_Stamp;
      Charge_Rate      :=
        Get_Charge_From_Voltage (Power_Info.V_Bat_1);
      LEDS.Set_Battery_Level (Charge_Rate);

      if Charge_Rate = 9 and then Is_Charging then
         State := Charged;

      elsif Is_Charging then
         State := Charging;

      elsif not Is_Pgood and not Is_Charging and
        Battery_Low_Time > PM_BAT_LOW_TIMEOUT
      then
         State := Low_Power;

      else
         State := Battery;
      end if;

      return State;
   end Get_State;

   --------------------
   -- Is_Discharging --
   --------------------

   function Is_Discharging return Boolean is
      State : Power_State;
   begin
      State := Get_State (Current_Power_Info);

      return State = Battery;
   end Is_Discharging;

   --------------------
   -- Set_Power_LEDs --
   --------------------

   procedure Set_Power_LEDs (State : Power_State) is
   begin
      case State is
         when Charging =>
            LEDS.Set_Battery_State (LEDS.Charging);
         when Charged =>
            LEDS.Set_Battery_State (LEDS.Charged);
         when Low_Power =>
            LEDS.Set_Battery_State (LEDS.Low_Power);
         when Battery =>
            LEDS.Set_Battery_State (LEDS.On_Battery);
         when others =>
            null;
      end case;
      --  TODO: find other led feedback for the other power states
   end Set_Power_LEDs;

   ---------------
   -- Task_Type --
   ---------------

   task body Task_Type is
      Now             : Ada.Real_Time.Time;
      Next_Period     : Ada.Real_Time.Time;
      New_Power_State : Power_State;

      use type Ada.Real_Time.Time;
   begin
      Now := Ada.Real_Time.Clock;
      Next_Period := Now + Ada.Real_Time.Milliseconds (500);

      Battery_Low_Time_Stamp := Now;
      Set_Power_LEDs (Current_Power_State);

      loop
         delay until Next_Period;

         Now := Ada.Real_Time.Clock;

         if Battery_Voltage > PM_BAT_LOW_VOLTAGE then
            Battery_Low_Time_Stamp := Now;
         end if;

         New_Power_State := Get_State (Current_Power_Info);

         --  Set the leds accordingly if the power state has changed
         if Current_Power_State /= New_Power_State then
            Set_Power_LEDs (New_Power_State);
            Current_Power_State := New_Power_State;
         end if;

         Next_Period := Next_Period + Ada.Real_Time.Milliseconds (500);
      end loop;
   end Task_Type;

end Power_Management;
