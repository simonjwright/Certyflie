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

package body Console is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      Ada.Synchronous_Task_Control.Set_True (Console_Access);
      Message_To_Print := CRTP.Create_Packet (CRTP.PORT_CONSOLE, 0);

      Is_Init := True;
   end Init;

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   ------------------
   -- Send_Message --
   ------------------

   procedure Send_Message (Has_Succeed : out Boolean) is
   begin
      CRTP.Send_Packet
        (CRTP.Get_Packet_From_Handler (Message_To_Print), Has_Succeed);

      --  Reset the CRTP packet data contained in the handler
      CRTP.Reset_Handler (Message_To_Print);
   end Send_Message;

   -----------
   -- Flush --
   -----------

   procedure Flush (Has_Succeed : out Boolean) is
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (Console_Access);
      Send_Message (Has_Succeed);
      Ada.Synchronous_Task_Control.Set_True (Console_Access);
   end Flush;

   --------------
   -- Put_Line --
   --------------

   procedure Put_Line
     (Message     : String;
      Has_Succeed : out Boolean)
   is
      Free_Bytes_In_Packet : Boolean := True;

      procedure Append_Character_Data is new CRTP.Append_Data (Character);

      procedure Put_Character (C : Character);
      procedure Put_Character (C : Character) is
      begin
         Append_Character_Data
           (Message_To_Print, C, Free_Bytes_In_Packet);

         if C = ASCII.LF or not Free_Bytes_In_Packet then
            Send_Message (Has_Succeed);
         end if;
      end Put_Character;
   begin
      for C of Message loop
         Put_Character (C);
      end loop;
      Put_Character (ASCII.LF);

      Send_Message (Has_Succeed);
   end Put_Line;

end Console;
