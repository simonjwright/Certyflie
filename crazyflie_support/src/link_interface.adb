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

with Crazyflie_Config;
with Radiolink;

package body Link_Interface is

   ---------------
   -- Init --
   ---------------

   procedure Init is
      use Crazyflie_Config;
   begin
      if Is_Init then
         return;
      end if;

      case LINK_LAYER_TYPE is
         when RADIO_LINK =>
            Radiolink.Init;
         when others =>
            --  Other link layers not implemented yet.
            null;
      end case;

      Is_Init := True;
   end Init;

   ----------------------
   -- Send_Packet --
   ----------------------

   procedure Send_Packet (Packet : CRTP.Packet;
                          One_Off : Boolean := False) is
      use Crazyflie_Config;
   begin
      case LINK_LAYER_TYPE is
         when RADIO_LINK =>
            Radiolink.Send_Packet (Packet, One_Off => One_Off);
         when others =>
            --  Other link layers not implemented yet.
            null;
      end case;
   end Send_Packet;

   ----------------------------------
   -- Receive_Packet_Blocking --
   ----------------------------------

   procedure Receive_Packet_Blocking (Packet : out CRTP.Packet) is
      use Crazyflie_Config;
   begin
      case LINK_LAYER_TYPE is
         when RADIO_LINK =>
            Radiolink.Receive_Packet_Blocking (Packet);
         when others =>
            --  Other link layers not implemented yet.
            null;
      end case;
   end Receive_Packet_Blocking;

end Link_Interface;
