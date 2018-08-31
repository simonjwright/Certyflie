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

with Syslink;

package body Platform_Service is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      CRTP.Register_Callback (CRTP.PORT_PLATFORM, Handler'Access);

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
   -- Handler --
   -------------

   procedure Handler (Packet : CRTP.Packet) is
      Has_Succeed : Boolean;
   begin
      case Packet.Channel is
         when Platform_Channel'Enum_Rep (PLAT_COMMAND) =>
            Command_Process
              (Packet.Data_1 (1), Packet.Data_1 (2 .. Packet.Data_1'Last));
            CRTP.Send_Packet (Packet, Has_Succeed);
         when others =>
            null;
      end case;
   end Handler;

   ---------------------
   -- Command_Process --
   ---------------------

   procedure Command_Process
     (Command : Types.T_Uint8;
      Data    : Types.T_Uint8_Array) is
      Sl_Packet : Syslink.Packet;
   begin
      case Command is
         when Platform_Command'Enum_Rep (SET_CONTINUOUS_WAVE) =>
            Sl_Packet.Slp_Type := Syslink.RADIO_CONTWAVE;
            Sl_Packet.Length := 1;
            Sl_Packet.Data (1) := Data (Data'First);
            Syslink.Send_Packet (Sl_Packet);
         when others =>
            null;
      end case;
   end Command_Process;

end Platform_Service;
