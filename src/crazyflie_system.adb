------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
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
pragma Warnings (Off, "is an internal GNAT unit");
with System.FreeRTOS.Tasks;
pragma Warnings (On, "is an internal GNAT unit");
with Hardfault_Handling;
pragma Unreferenced (Hardfault_Handling);

with Communication;
with Commander;
with IMU;
with LEDS;
with Log;
with Memory;
with Motors;
with Parameter;
with Power_Management;
with Stabilizer;
with Types;

package body Crazyflie_System is

   procedure Initialize_System_Parameter_Logging;
   procedure Self_Test_Status (OK : Boolean) with Inline;

   -----------------
   -- System_Init --
   -----------------

   procedure System_Init is
   begin
      if Is_Init then
         return;
      end if;
      --  Module initialization.

      --  Initialize LEDs, power management, sensors and actuators.
      LEDS.Init;
      Motors.Init;
      IMU.Init (Use_Mag    => True,
                DLPF_256Hz => False);

      --  Initialize communication related modules.
      Communication.Init;

      --  Initialize power management module.
      Power_Management.Init;

      --  Initialize memory module.
      Memory.Init;

      --  Initialize logging.
      Log.Init;

      --  Initialize parameters.
      Parameter.Init;
      Initialize_System_Parameter_Logging;

      --  Initialize high level modules.
      Commander.Init;
      Stabilizer.Init;

      Is_Init := True;
   end System_Init;

   ----------------------
   -- System_Self_Test --
   ----------------------

   function System_Self_Test return Boolean is
      Self_Test_Passed : Boolean;
   begin
      LEDS.Set_System_State (LEDS.Self_Test);
      Self_Test_Passed := LEDS.Test;
      Self_Test_Passed := Self_Test_Passed and Motors.Test;
      Self_Test_Passed := Self_Test_Passed and IMU.Test;
      Self_Test_Passed := Self_Test_Passed and Communication.Test;
      Self_Test_Passed := Self_Test_Passed and Memory.Test;
      Self_Test_Passed := Self_Test_Passed and Commander.Test;
      Self_Test_Passed := Self_Test_Passed and Stabilizer.Test;

      if Self_Test_Passed then
         LEDS.Set_System_State (LEDS.Calibrating);

         if IMU.Calibrate_6 then
            LEDS.Set_System_State (LEDS.Ready);
         else
            LEDS.Set_System_State (LEDS.Failure);
         end if;

      elsif not Self_Test_Passed then
         LEDS.Set_System_State (LEDS.Failure);
      end if;

      Self_Test_Status (OK => Self_Test_Passed);

      return Self_Test_Passed;
   end System_Self_Test;

   -----------------
   -- System_Loop --
   -----------------

   procedure System_Loop is
      use Ada.Real_Time;
      use Types;
      Attitude_Update_Counter : T_Uint32 := 0;
      Alt_Hold_Update_Counter : T_Uint32 := 0;
      Next_Period             : Time;
   begin
      Next_Period := Clock + IMU.UPDATE_DT_MS;

      loop
         delay until Next_Period;
         Stabilizer.Control_Loop (Attitude_Update_Counter,
                                  Alt_Hold_Update_Counter);

         Next_Period := Next_Period + IMU.UPDATE_DT_MS;
      end loop;
   end System_Loop;

   -------------------------
   -- Last_Chance_Handler --
   -------------------------

   procedure Last_Chance_Handler (Message : Interfaces.C.Strings.chars_ptr;
                                  Line : Integer)
   is
      pragma Unreferenced (Message, Line);
      use Ada.Real_Time;
   begin
      --  As like as not, we've got some interrupt-related error. Of
      --  course, that may prevent us ever getting here.
      System.FreeRTOS.Tasks.Disable_Interrupts;

      Motors.Reset;
      LEDS.Reset_All;

      --  It's a no-return procedure...
      loop
         LEDS.Toggle (LEDS.Red_L);
         delay until Clock + Milliseconds (1_000);
      end loop;
   end Last_Chance_Handler;

   ------------------------------
   -- System Parameter Logging --
   ------------------------------

   System_Group_ID : Natural := 0;
   System_Group_Created : Boolean;

   Self_Test_Passed : Boolean := True
   with Convention => C;

   pragma Assert (Self_Test_Passed'Size = 8);

   procedure Initialize_System_Parameter_Logging is
      use Parameter;
   begin
      Create_Parameter_Group (Name        => "system",
                              Group_ID    => System_Group_ID,
                              Has_Succeed => System_Group_Created);

      if System_Group_Created then
         declare
            Dummy          : Boolean;
            Parameter_Type : constant Parameter.Parameter_Variable_Type
              := (Size      => One_Byte,
                  Floating  => False,
                  Signed    => False,
                  Read_Only => True,
                  others    => <>);
         begin
            Parameter.Append_Parameter_Variable_To_Group
              (System_Group_ID,
               Name           => "selftestPassed",
               Parameter_Type => Parameter_Type,
               Variable       => Self_Test_Passed'Address,
               Has_Succeed    => Dummy);
         end;
      end if;
   end Initialize_System_Parameter_Logging;

   procedure Self_Test_Status (OK : Boolean) is
   begin
      Self_Test_Passed := OK;
   end Self_Test_Status;

end Crazyflie_System;
