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

with Safety;

package body Commander
with SPARK_Mode,
  Refined_State => (Commander_State  => (Is_Init,
                                         Is_Inactive,
                                         Alt_Hold_Mode,
                                         Alt_Hold_Mode_Old,
                                         Thrust_Locked,
                                         Side,
                                         Target_Val,
                                         Last_Update))
is
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

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init
     with SPARK_Mode => Off
   is
   begin
      if Is_Init then
         return;
      end if;

      Last_Update := Ada.Real_Time.Clock;
      CRTP.Register_Callback
        (CRTP.PORT_COMMANDER, CRTP_Handler'Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   ------------------------------
   -- Get_Commands_From_Packet --
   ------------------------------

   function Get_Commands_From_Packet
     (Packet : CRTP.Packet) return CRTP_Values
   is
      Commands     : CRTP_Values;
      Handler      : CRTP.Packet_Handler;
      Has_Succeed  : Boolean;
   begin
      Handler := CRTP.Get_Handler_From_Packet (Packet);

      pragma Warnings (Off, "unused assignment",
                       Reason => "Has_Succeed can't be equal to false here");
      CRTP_Get_Float_Data (Handler, 1, Commands.Roll, Has_Succeed);
      CRTP_Get_Float_Data (Handler, 5, Commands.Pitch, Has_Succeed);
      CRTP_Get_Float_Data (Handler, 9, Commands.Yaw, Has_Succeed);
      CRTP_Get_T_Uint16_Data (Handler, 13, Commands.Thrust, Has_Succeed);
      pragma Warnings (On, "unused assignment");

      return Commands;
   end Get_Commands_From_Packet;

   ------------------
   -- CRTP_Handler --
   ------------------

   procedure CRTP_Handler (Packet : CRTP.Packet) is
      use type Types.T_Uint16;
   begin
      Side := not Side;
      Target_Val (Side) := Get_Commands_From_Packet (Packet);

      if Target_Val (Side).Thrust = 0 then
         Thrust_Locked := False;
      end if;

      Watchdog_Reset;
   end CRTP_Handler;

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
