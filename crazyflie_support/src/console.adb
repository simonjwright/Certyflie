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

with Config;
with CRTP;

package body Console is

   Is_Init : Boolean := False;

   --  PO to buffer message text
   protected Message_Buffer is
      entry Get (Str : out String; Last : out Positive);
      procedure Put (Str : String);
      procedure Put (C : Character);
      procedure Flush;
   private
      Buffer : String (1 .. 2048);
      Last   : Natural := 0;
      Not_Empty  : Boolean := False; -- Ravenscar requires simple barrier
   end Message_Buffer;

   task Console_Task
     with Priority => Config.CONSOLE_TASK_PRIORITY;

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;
      Is_Init := True;
   end Init;

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   -----------
   -- Flush --
   -----------

   procedure Flush is
   begin
      Message_Buffer.Flush;
   end Flush;

   ---------
   -- Put --
   ---------

   procedure Put (Message : String)
   is
   begin
      Message_Buffer.Put (Message);
   end Put;

   --------------
   -- Put_Line --
   --------------

   procedure Put_Line (Message : String)
   is
   begin
      Message_Buffer.Put (Message & ASCII.LF);
   end Put_Line;

   procedure Put_Line (Message : String; Dummy : out Boolean)
   is
   begin
      Dummy := True;
      Message_Buffer.Put (Message & ASCII.LF);
   end Put_Line;

   --------------------
   -- Message_Buffer --
   --------------------

   protected body Message_Buffer is
      entry Get (Str : out String; Last : out Positive) when Not_Empty is
         pragma Assert (Str'First = 1, "non-standard string");
         Remaining_Characters : constant Natural
           := Natural'Max (Message_Buffer.Last - Str'Last, 0);
      begin
         Last := Message_Buffer.Last - Remaining_Characters;
         Str (1 .. Last) := Buffer (1 .. Last);
         Buffer (1 .. Remaining_Characters) :=
           Buffer (Message_Buffer.Last + 1 - Remaining_Characters
                     .. Message_Buffer.Last);
         Message_Buffer.Last := Remaining_Characters;
         Not_Empty := Message_Buffer.Last > 0;
      end Get;

      procedure Put (Str : String) is
      begin
         Buffer (Last + 1 .. Last + Str'Length) := Str;  -- fingers crossed
         Last := Last + Str'Length;
         Not_Empty := True;
      end Put;

      procedure Put (C : Character) is
      begin
         Last := Last + 1;
         Buffer (Last) := C;
         Not_Empty := True;
      end Put;

      procedure Flush is
      begin
         Put (ASCII.NUL);
      end Flush;
   end Message_Buffer;

   task body Console_Task is
      procedure Append_Character
        is new CRTP.Append_Data_If_Room (Character);
      procedure Send_Message;

      Message_To_Print : CRTP.Packet_Handler
        := CRTP.Create_Packet (CRTP.PORT_CONSOLE, 0);

      procedure Send_Message is
      begin
         CRTP.Send_Packet
           (CRTP.Get_Packet_From_Handler (Message_To_Print),
            One_Off => True);
         CRTP.Reset_Handler (Message_To_Print);
      end Send_Message;

      Buff : String (1 .. CRTP.MAX_DATA_SIZE);
      Last : Positive;
   begin
      loop
         Message_Buffer.Get (Buff, Last);
         for C in 1 .. Last loop
            declare
               Ch : constant Character := Buff (C);
               Appended : Boolean;
            begin
               if Ch = ASCII.NUL -- which signals to flush
               then
                  Send_Message;
               else
                  Append_Character (Message_To_Print, Ch, Appended);
                  if not Appended
                  then
                     Send_Message;
                     Append_Character (Message_To_Print, Ch, Appended);
                     pragma Assert (Appended);
                  end if;
                  if Ch = ASCII.LF then
                     Send_Message;
                  end if;
               end if;
            end;
         end loop;
      end loop;
   end Console_Task;

end Console;
