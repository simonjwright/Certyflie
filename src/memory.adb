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
with Types;

package body Memory is

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      CRTP.Register_Callback (CRTP.PORT_MEM, CRTP_Handler'Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   --  Private procedures and functions

   ------------------
   -- CRTP_Handler --
   ------------------

   procedure CRTP_Handler (Packet : CRTP.Packet)
   is
      ------------------------------------
      -- CRTP_Channel_To_Memory_Channel --
      ------------------------------------

      function CRTP_Channel_To_Memory_Channel is new Ada.Unchecked_Conversion
        (CRTP.Channel_T, Memory_Channel);
      Channel : Memory_Channel;
   begin
      Channel := CRTP_Channel_To_Memory_Channel (Packet.Channel);

      case Channel is
         when SETTINGS_CH =>
            Settings_Process (Packet);
         when READ_CH =>
            null;
         when WRITE_CH =>
            null;
      end case;
   end CRTP_Handler;

   ----------------------
   -- Settings_Process --
   ----------------------

   procedure Settings_Process (Packet : CRTP.Packet)
   is
      -------------------------------
      -- T_Uint8_To_Memory_Command --
      -------------------------------

      function T_Uint8_To_Memory_Command is new Ada.Unchecked_Conversion
        (Types.T_Uint8, Memory_Command);

      ------------------------------
      -- CRTP_Append_T_Uint8_Data --
      ------------------------------

      procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
        (Types.T_Uint8);

      Command        : Memory_Command;
      Packet_Handler : CRTP.Packet_Handler;
      Has_Succeed    : Boolean;
      pragma Unreferenced (Has_Succeed);
   begin
      Command := T_Uint8_To_Memory_Command (Packet.Data_1 (1));

      Packet_Handler := CRTP.Create_Packet
        (CRTP.PORT_MEM, Memory_Channel'Enum_Rep (SETTINGS_CH));
      CRTP_Append_T_Uint8_Data
        (Packet_Handler,
         Memory_Command'Enum_Rep (Command),
         Has_Succeed);

      case Command is
         when CMD_GET_NBR =>
            --  TODO: for now we just send 0 instead of the real number
            --  of One Wire memories + EEPROM memory.
            CRTP_Append_T_Uint8_Data
              (Packet_Handler,
               0,
               Has_Succeed);
            CRTP.Send_Packet
              (CRTP.Get_Packet_From_Handler (Packet_Handler),
               Has_Succeed);
         when CMD_GET_INFO =>
            null;
      end case;
   end Settings_Process;

end Memory;
