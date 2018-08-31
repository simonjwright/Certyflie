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

with Ada.Real_Time;

with CRTP;
with Types;
pragma Elaborate_All (CRTP);

package Commander
with SPARK_Mode,
  Abstract_State => Commander_State
is

   --  Types

   --  Type of the commands given by the pilot.
   --  Can be an angle rate, or an angle.
   type RPY_Type is (RATE, ANGLE);
   for RPY_Type use (RATE => 0, ANGLE => 1);
   for RPY_Type'Size use Integer'Size;

   --  Type used to represent different commands
   --  received in a CRTP packet sent from the client.
   type CRTP_Values is record
      Roll   : Types.T_Degrees := 0.0;
      Pitch  : Types.T_Degrees := 0.0;
      Yaw    : Types.T_Degrees := 0.0;
      Thrust : Types.T_Uint16 := 0;
   end record;
   pragma Pack (CRTP_Values);

   --  Procedures and functions

   --  Initizalize the Commander module.
   procedure Init;

   --  Test if the Commander module is initialized.
   function Test return Boolean;

   --  Handler called when a CRTP packet is received in the commander
   --  port queue.
   procedure CRTP_Handler (Packet : CRTP.Packet);

   --  Get the commands from the pilot.
   procedure Get_RPY
     (Euler_Roll_Desired  : out Types.T_Degrees;
      Euler_Pitch_Desired : out Types.T_Degrees;
      Euler_Yaw_Desired   : out Types.T_Degrees)
     with
       Global => (Input => Commander_State);

   --  Get the commands types by default or from the client..
   procedure Get_RPY_Type
     (Roll_Type  : out RPY_Type;
      Pitch_Type : out RPY_Type;
      Yaw_Type   : out RPY_Type)
     with
       Global => null;

   --  Get the thrust from the pilot.
   procedure Get_Thrust (Thrust : out Types.T_Uint16)
     with
       Global => (Input  => Ada.Real_Time.Clock_Time,
                  In_Out => (CRTP.CRTP_State,
                             Commander_State));

   --  Get Alt Hold Mode parameters from the pilot..
   procedure Get_Alt_Hold
     (Alt_Hold        : out Boolean;
      Set_Alt_Hold    : out Boolean;
      Alt_Hold_Change : out Float);

   --  Cut the trust when inactivity time has been during for too long.
   procedure Watchdog
     with
       Global => (Input  => Ada.Real_Time.Clock_Time,
                  In_Out => (CRTP.CRTP_State,
                             Commander_State));

private

   --  Global variables and constants

   COMMANDER_WDT_TIMEOUT_STABILIZE : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (500);
   COMMANDER_WDT_TIMEOUT_SHUTDOWN  : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (1000);

   MIN_THRUST        : constant := 1000;
   MAX_THRUST        : constant := 60_000;
   ALT_HOLD_THRUST_F : constant := 32_767.0;

   Is_Init           : Boolean := False
     with
       Part_Of => Commander_State;
   Is_Inactive       : Boolean := True
     with
       Part_Of => Commander_State;
   Alt_Hold_Mode     : Boolean := False
     with
       Part_Of => Commander_State;
   Alt_Hold_Mode_Old : Boolean := False
     with
       Part_Of => Commander_State;
   Thrust_Locked     : Boolean := True
     with
       Part_Of => Commander_State;
   Side              : Boolean := False
     with
       Part_Of => Commander_State;

   --  Container for the commander values received via CRTP.
   Target_Val : array (Boolean) of CRTP_Values
     with
       Part_Of => Commander_State;

   Last_Update : Ada.Real_Time.Time
     with
       Part_Of => Commander_State;

   --  Procedures and functions

   --  Reset the watchdog by assigning the Clock current value to Last_Update
   --  variable.
   procedure Watchdog_Reset;
   pragma Inline (Watchdog_Reset);

   --  Get inactivity time since last update.
   function Get_Inactivity_Time return Ada.Real_Time.Time_Span
     with
       Volatile_Function;
   pragma Inline (Get_Inactivity_Time);

   --  Get target values from a received CRTP packet
   function Get_Commands_From_Packet
     (Packet : CRTP.Packet) return CRTP_Values;

   --  Get Float data from a CRTP Packet.
   procedure CRTP_Get_Float_Data is new CRTP.Get_Data (Float);

   --  Get T_Uint16 data from a CRTP Packet.
   procedure CRTP_Get_T_Uint16_Data is new CRTP.Get_Data (Types.T_Uint16);

end Commander;
