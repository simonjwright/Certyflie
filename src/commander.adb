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
with Console;
with CRTP;
with Safety;

package body Commander is

   --  [many declarations moved from the spec]

   --  The commands that may be received in a SETPOINT_GENERIC packet.
   type Setpoint_Packet_Kind is
     (Stop,
      World_Velocity,
      Z_Distance,
      cppEmu,  -- emulation of CPPM channels - ?
      Alt_Hold,
      Hover,
      Full_State,
      Position)
   with Size => 8;
   for Setpoint_Packet_Kind use
     (Stop           => 0,
      World_Velocity => 1,
      Z_Distance     => 2,
      cppEmu         => 3,
      Alt_Hold       => 4,
      Hover          => 5,
      Full_State     => 6,
      Position       => 7);

   --  Type used to represent different commands
   --  received in a CRTP packet sent from the client.
   type CRTP_Values is record
      Roll   : Types.T_Degrees := 0.0;
      Pitch  : Types.T_Degrees := 0.0;
      Yaw    : Types.T_Degrees := 0.0;
      Thrust : Types.T_Uint16 := 0;
   end record;
   pragma Pack (CRTP_Values);

   --  Global variables and constants

   COMMANDER_WDT_TIMEOUT_STABILIZE : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (500);
   COMMANDER_WDT_TIMEOUT_SHUTDOWN  : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (1000);

   MIN_THRUST        : constant := 1000; pragma Unreferenced (MIN_THRUST);
   MAX_THRUST        : constant := 60_000;
   ALT_HOLD_THRUST_F : constant := 32_767.0;

   Is_Init           : Boolean := False;
   Is_Inactive       : Boolean := True; pragma Unreferenced (Is_Inactive);
   Alt_Hold_Mode     : Boolean := False;
   Alt_Hold_Mode_Old : Boolean := False;
   Thrust_Locked     : Boolean := True;
   Side              : Boolean := False;

   --  Container for the commander values received via CRTP.
   Target_Val : array (Boolean) of CRTP_Values;

   Last_Update : Ada.Real_Time.Time;

   --  Procedures and functions

   --  Handler called when a SETPOINT CRTP packet is received in the
   --  commander port queue.
   procedure CRTP_Setpoint_Handler (Packet : CRTP.Packet);

   --  Handler called when a SETPOINT_GENERIC CRTP packet is received
   --  in the commander port queue.
   procedure CRTP_Setpoint_Generic_Handler (Packet : CRTP.Packet);

   --  Reset the watchdog by assigning the Clock current value to Last_Update
   --  variable.
   procedure Watchdog_Reset;
   pragma Inline (Watchdog_Reset);

   --  Get inactivity time since last update.
   function Get_Inactivity_Time return Ada.Real_Time.Time_Span
     with
       Volatile_Function;
   pragma Inline (Get_Inactivity_Time);

   --  Get Float data from a CRTP Packet.
   procedure CRTP_Get_Float_Data is new CRTP.Get_Data (Float);

   --  Get T_Uint16 data from a CRTP Packet.
   procedure CRTP_Get_T_Uint16_Data is new CRTP.Get_Data (Types.T_Uint16);

   --  Private procedures and functions

   --------------------
   -- Watchdog_Reset --
   --------------------

   procedure Watchdog_Reset is
   begin
      CRTP.Set_Is_Connected (True);

      Last_Update := Ada.Real_Time.Clock;
   end Watchdog_Reset;

   -------------------------
   -- Get_Inactivity_Time --
   -------------------------

   function Get_Inactivity_Time return Ada.Real_Time.Time_Span
   is
      Current_Time : constant Ada.Real_Time.Time := Ada.Real_Time.Clock;
      use type Ada.Real_Time.Time;
   begin
      return Current_Time - Last_Update;
   end Get_Inactivity_Time;

   --------------
   -- Watchdog --
   --------------

   procedure Watchdog is
      Used_Side : Boolean;
      Time_Since_Last_Update : Ada.Real_Time.Time_Span;

      use type Ada.Real_Time.Time_Span;
   begin
      --  To prevent the change of Side value when this is called
      Used_Side := Side;

      Time_Since_Last_Update := Get_Inactivity_Time;

      if Time_Since_Last_Update > COMMANDER_WDT_TIMEOUT_STABILIZE then
         CRTP.Set_Is_Connected (False);

         Target_Val (Used_Side).Roll := 0.0;
         Target_Val (Used_Side).Pitch := 0.0;
         Target_Val (Used_Side).Yaw := 0.0;
      end if;

      if Time_Since_Last_Update > COMMANDER_WDT_TIMEOUT_SHUTDOWN then
         Target_Val (Used_Side).Thrust := 0;
         --  TODO: set the alt hold mode variable to false
         Alt_Hold_Mode := False;
         Is_Inactive := True;
         Thrust_Locked := True;
      else
         Is_Inactive := False;
      end if;
   end Watchdog;

   ---------------------------
   -- CRTP_Setpoint_Handler --
   ---------------------------

   procedure CRTP_Setpoint_Handler (Packet : CRTP.Packet) is
      Handler  : constant CRTP.Packet_Handler
        := CRTP.Get_Handler_From_Packet (Packet);
      use type Types.T_Uint16;

      --  Target_Val is a swing buffer.
      Target : CRTP_Values renames Target_Val (not Side);
   begin
      --  Write the 'other side' of the swing buffer.
      CRTP_Get_Float_Data (Handler, 1, Target.Roll);
      CRTP_Get_Float_Data (Handler, 5, Target.Pitch);
      CRTP_Get_Float_Data (Handler, 9, Target.Yaw);
      CRTP_Get_T_Uint16_Data (Handler, 13, Target.Thrust);
      --  Swap sides.
      Side := not Side;

      if Target_Val (Side).Thrust = 0 then
         Thrust_Locked := False;
      end if;

      Watchdog_Reset;
   end CRTP_Setpoint_Handler;

   -----------------------------------
   -- CRTP_Setpoint_Generic_Handler --
   -----------------------------------

   procedure CRTP_Setpoint_Generic_Handler (Packet : CRTP.Packet) is
      procedure Get_Setpoint_Packet_Kind
        is new CRTP.Get_Data (Setpoint_Packet_Kind);
      Handler : constant CRTP.Packet_Handler
        := CRTP.Get_Handler_From_Packet (Packet);
      Kind    : Setpoint_Packet_Kind;
   begin
      Get_Setpoint_Packet_Kind (Handler, 1, Kind);
      Console.Put_Line ("Received Generic Setpoint kind: " & Kind'Image);
      Watchdog_Reset;
   end CRTP_Setpoint_Generic_Handler;

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      Last_Update := Ada.Real_Time.Clock;
      CRTP.Register_Callback
        (CRTP.PORT_SETPOINT, CRTP_Setpoint_Handler'Access);
      CRTP.Register_Callback
        (CRTP.PORT_SETPOINT_GENERIC, CRTP_Setpoint_Generic_Handler'Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   -------------
   -- Get_RPY --
   -------------

   procedure Get_RPY
     (Euler_Roll_Desired  : out Types.T_Degrees;
      Euler_Pitch_Desired : out Types.T_Degrees;
      Euler_Yaw_Desired   : out Types.T_Degrees)
   is
      Used_Side : Boolean;
   begin
      --  To prevent the change of Side value when this is called
      Used_Side := Side;

      Euler_Roll_Desired := Target_Val (Used_Side).Roll;
      Euler_Pitch_Desired := Target_Val (Used_Side).Pitch;
      Euler_Yaw_Desired := Target_Val (Used_Side).Yaw;
   end Get_RPY;

   ------------------
   -- Get_RPY_Type --
   ------------------

   procedure Get_RPY_Type
     (Roll_Type  : out RPY_Type;
      Pitch_Type : out RPY_Type;
      Yaw_Type   : out RPY_Type) is
   begin
      Roll_Type := ANGLE;
      Pitch_Type := ANGLE;
      Yaw_Type := RATE;
   end Get_RPY_Type;

   ----------------
   -- Get_Thrust --
   ----------------

   procedure Get_Thrust (Thrust : out Types.T_Uint16) is
      Raw_Thrust : Types.T_Uint16;
   begin
      Raw_Thrust := Target_Val (Side).Thrust;

      if Thrust_Locked then
         Thrust := 0;
      else
         Thrust := Safety.Saturate (Raw_Thrust, 0, MAX_THRUST);
      end if;

      Watchdog;
   end Get_Thrust;

   ------------------
   -- Get_Alt_Hold --
   ------------------

   procedure Get_Alt_Hold
     (Alt_Hold        : out Boolean;
      Set_Alt_Hold    : out Boolean;
      Alt_Hold_Change : out Float) is
   begin
      Alt_Hold := Alt_Hold_Mode;
      Set_Alt_Hold := Alt_Hold_Mode and not Alt_Hold_Mode_Old;
      Alt_Hold_Change :=
        (if Alt_Hold_Mode then
           (Float (Target_Val (Side).Thrust) - ALT_HOLD_THRUST_F)
           / ALT_HOLD_THRUST_F
         else
            0.0);
      Alt_Hold_Mode_Old := Alt_Hold_Mode;
   end Get_Alt_Hold;

end Commander;
