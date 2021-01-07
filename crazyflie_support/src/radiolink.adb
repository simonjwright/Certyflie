------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
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

with Ada.Unchecked_Conversion;

with Crazyflie_Config;

with LEDS;

package body Radiolink is

   -- Local procedures --

   --  If the received syslink packet is RADIO_RAW (not RADIO_RSSI,
   --  which we don't use?), send to CRTP. If we receive a CRTP
   --  packet, send it out via Syslink.
   procedure Syslink_Dispatch (Rx_Sl_Packet : Syslink.Packet);

   Red_L   : LEDS.Flasher (LEDS.Red_L'Access);
   --  Indicate we've transmitted a packet.

   Green_L   : LEDS.Flasher (LEDS.Green_L'Access);
   --  Indicate we've received a packet.

   --------------------
   -- Init --
   --------------------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      Syslink.Init;
      Syslink.Register_Callback (Syslink.RADIO_GROUP, Syslink_Dispatch'Access);

      Set_Channel (Crazyflie_Config.RADIO_CHANNEL);
      Set_Data_Rate (Crazyflie_Config.RADIO_DATARATE);
      Set_Address (Crazyflie_Config.RADIO_ADDRESS);

      Is_Init := True;
   end Init;

   -----------------------------
   -- Set_Data_Rate --
   -----------------------------

   procedure Set_Data_Rate (Data_Rate : Types.T_Uint8)
   is
      Sl_Packet : Syslink.Packet;
   begin
      Sl_Packet.Slp_Type := Syslink.RADIO_DATARATE;
      Sl_Packet.Length := 1;
      Sl_Packet.Data (1) := Data_Rate;
      Syslink.Send_Packet (Sl_Packet);
   end Set_Data_Rate;

   ---------------------------
   -- Set_Channel --
   ---------------------------

   procedure Set_Channel (Channel : Types.T_Uint8)
   is
      Sl_Packet : Syslink.Packet;
   begin
      Sl_Packet.Slp_Type := Syslink.RADIO_CHANNEL;
      Sl_Packet.Length := 1;
      Sl_Packet.Data (1) := Channel;
      Syslink.Send_Packet (Sl_Packet);
   end Set_Channel;

   ---------------------------
   -- Set_Address --
   ---------------------------

   procedure Set_Address (Address : Types.T_Uint64)
   is
      Sl_Packet : Syslink.Packet;
      subtype As_Bytes is Types.T_Uint8_Array (1 .. 8);
      function Convert is new Ada.Unchecked_Conversion (Types.T_Uint64,
                                                        As_Bytes);
      Address_As_Bytes : constant As_Bytes := Convert (Address);
   begin
      Sl_Packet.Slp_Type := Syslink.RADIO_ADDRESS;
      Sl_Packet.Length := 5;
      Sl_Packet.Data (1 .. 5) := Address_As_Bytes (1 .. 5);
      Syslink.Send_Packet (Sl_Packet);
   end Set_Address;

   ---------------------------------------
   -- Receive_Packet_Blocking --
   ---------------------------------------

   procedure Receive_Packet_Blocking (Packet : out CRTP.Packet) is
   begin
      Rx_Queue.Await_Item_To_Dequeue (Packet);
   end Receive_Packet_Blocking;

   -----------------
   -- Send_Packet --
   -----------------

   function Send_Packet (Packet : CRTP.Packet) return Boolean
   is
      Sl_Packet : Syslink.Packet;
      Has_Succeed : Boolean;

      use type Types.T_Uint8;
   begin
      Sl_Packet.Length := Packet.Size + 1;
      Sl_Packet.Slp_Type := Syslink.RADIO_RAW;
      Sl_Packet.Data (1 .. 31) := Packet.Raw_Pkt;

      --  Try to enqueue the Syslink packet
      Tx_Queue.Enqueue_Item (Sl_Packet, Has_Succeed);

      return Has_Succeed;
   end Send_Packet;

   --------------------------------
   -- Syslink_Dispatch --
   --------------------------------

   procedure Syslink_Dispatch (Rx_Sl_Packet : Syslink.Packet)
   is
      Tx_Sl_Packet   : Syslink.Packet;
      Rx_CRTP_Packet : CRTP.Packet;
      Has_Succeed    : Boolean;

      use type Types.T_Uint8;
      use type Syslink.Packet_Type;
   begin
      if Rx_Sl_Packet.Slp_Type = Syslink.RADIO_RAW then
         Rx_CRTP_Packet.Size := Rx_Sl_Packet.Length - 1;
         Rx_CRTP_Packet.Header := Rx_Sl_Packet.Data (1);
         Rx_CRTP_Packet.Data_2 :=
           CRTP.Data_T (Rx_Sl_Packet.Data (2 .. 31));
         --  should really only copy significant bytes, but ...

         --  Enqueue the received packet
         Rx_Queue.Enqueue_Item (Rx_CRTP_Packet, Has_Succeed);
         if Has_Succeed then
            Green_L.Set;
         end if;

         --  If a radio packet is received, one can be sent
         Tx_Queue.Dequeue_Item (Tx_Sl_Packet, Has_Succeed);

         if Has_Succeed then
            Red_L.Set;
            Syslink.Send_Packet (Tx_Sl_Packet);
         end if;
      elsif Rx_Sl_Packet.Slp_Type = Syslink.RADIO_RSSI then
         --  Extract RSSI sample sent from Radio
         RSSI := Rx_Sl_Packet.Data (1);
      end if;
   end Syslink_Dispatch;

end Radiolink;
