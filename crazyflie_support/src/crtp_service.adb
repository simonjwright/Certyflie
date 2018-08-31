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

package body CRTP_Service is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      CRTP.Register_Callback (CRTP.PORT_LINK,
                              Handler'Access);

      Is_Init := True;
   end Init;

   -------------
   -- Handler --
   -------------

   procedure Handler (Packet : CRTP.Packet)
   is
      Cmd         : Command;
      Tx_Packet   : CRTP.Packet := Packet;
      Has_Succeed : Boolean;

      -----------------------------
      -- CRTP_Channel_To_Command --
      -----------------------------

      function CRTP_Channel_To_Command is
        new Ada.Unchecked_Conversion (CRTP.Channel_T, Command);
   begin
      Cmd := CRTP_Channel_To_Command (Packet.Channel);

      case Cmd is
         when Link_Echo =>
            CRTP.Send_Packet (Tx_Packet, Has_Succeed);
         when Link_Source =>
            Tx_Packet.Size := CRTP.MAX_DATA_SIZE;
            Tx_Packet.Data_1 := (others => 0);
            CRTP.Send_Packet (Tx_Packet, Has_Succeed);
         when others =>
            --  Null packets
            null;
      end case;
   end Handler;

end CRTP_Service;
