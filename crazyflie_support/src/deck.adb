------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--        Copyright (C) 2020, Simon Wright <simon@pushface.org>             --
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

with Flow_Deck;
with One_Wire;
with Types;

with Semihosting;

package body Deck is

   Is_Init : Boolean := False;

   procedure Init is
      --  Going to check the number of One_Wire targets, and (if one)
      --  try to initialize the flow deck.
      OW_Scan_OK : Boolean;
      Number_Of_OW_Targets : Types.T_Uint8;
      use type Types.T_Uint8;
   begin
      One_Wire.Init;
      if One_Wire.Test then
         OW_Scan_OK := One_Wire.Scan
           (Number_Of_Targets => Number_Of_OW_Targets);
         if OW_Scan_OK then
            if Number_Of_OW_Targets = 1 then
               declare
                  Serial : One_Wire.Target_Serial;
               begin
                  if One_Wire.Get_Info (0, Serial) then
                     Semihosting.Log_Line ("One_Wire.Get_Info:");
                     for J of Serial loop
                        Semihosting.Log (" " & J'Image);
                     end loop;
                     Semihosting.Log_New_Line;
                  else
                     Semihosting.Log_Line ("One_Wire.Get_Info failed");
                  end if;
               end;
               Flow_Deck.Init;
               if not Flow_Deck.Test then
                  Semihosting.Log_Line
                    ("Deck: Flow_Deck initialization failed");
               end if;
            else
               Semihosting.Log_Line
                 ("Number of OW targets:"
                    & Number_Of_OW_Targets'Image
                    & " should be 1 for this Flow Deck code");
            end if;
         else
            Semihosting.Log_Line ("Deck: One_Wire.Scan failed");
         end if;
      else
         Semihosting.Log_Line ("Deck: One_Wire not initialized");
      end if;
      Is_Init := True;
   end Init;

   function Test return Boolean is (Is_Init);

end Deck;
