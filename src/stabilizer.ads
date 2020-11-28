------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2016, AdaCore                     --
--           Copyright (C) 2020, Simon Wright <simon@pushface.org>          --
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

package Stabilizer is

   --  Procedures and functions

   --  Initialize the stabilizer module.
   procedure Init;

   --  Test if stabilizer module is correctly initialized.
   function Test return Boolean;

   --  Main function of the stabilization system. Get the commands, give them
   --  to the PIDs, and get the output to control the actuators.
   --
   --  The two counters are passed here to avoid having to provide static
   --  variables in the body (not a good reason, IMO)
   procedure Control_Loop
     (Attitude_Update_Counter : in out Types.T_Uint32;
      Alt_Hold_Update_Counter : in out Types.T_Uint32);

private

end Stabilizer;
