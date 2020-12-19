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

with Types;

package Commander is

   --  Types

   --  Type of the commands given by the pilot.
   --  Can be an angle rate, or an angle.
   type RPY_Type is (RATE, ANGLE);

   --  Procedures and functions

   --  Initizalize the Commander module.
   procedure Init;

   --  Test if the Commander module is initialized.
   function Test return Boolean;

   --  Get the commands from the pilot.
   procedure Get_RPY
     (Euler_Roll_Desired  : out Types.T_Degrees;
      Euler_Pitch_Desired : out Types.T_Degrees;
      Euler_Yaw_Desired   : out Types.T_Degrees);

   --  Get the command types by default or from the client.
   procedure Get_RPY_Type
     (Roll_Type  : out RPY_Type;
      Pitch_Type : out RPY_Type;
      Yaw_Type   : out RPY_Type);

   --  Get the thrust from the pilot.
   procedure Get_Thrust (Thrust : out Types.T_Uint16);

   --  Get Alt Hold Mode parameters from the pilot.
   procedure Get_Alt_Hold
     (Alt_Hold        : out Boolean;
      Set_Alt_Hold    : out Boolean;
      Alt_Hold_Change : out Float);

   --  Cut the thrust when inactivity time has been during for too long.
   procedure Watchdog;

private

   for RPY_Type use (RATE => 0, ANGLE => 1);
   for RPY_Type'Size use Integer'Size;

end Commander;
