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

--  Implements the link between the two Crazyflie MCUs.

with Ada.Synchronous_Task_Control;
with Ada.Unchecked_Conversion;
with System;

with Types;
private with UART_Syslink;

package Syslink is

   --  Global variables and constants

   --  Size of Syslink packet data.
   MTU      : constant := 31;

   --  Types

   --  Syslink packet types.
   type Packet_Type is (RADIO_RAW,
                        RADIO_CHANNEL,
                        RADIO_DATARATE,
                        RADIO_CONTWAVE,
                        RADIO_RSSI,
                        RADIO_ADDRESS,
                        PM_SOURCE,
                        PM_ONOFF_SWITCHOFF,
                        PM_BATTERY_VOLTAGE,
                        PM_BATTERY_STATE,
                        PM_BATTERY_AUTOUPDATE,
                        OW_SCAN,
                        OW_GETINFO,
                        OW_READ,
                        OW_WRITE);
   for Packet_Type use (RADIO_RAW             => 16#00#,
                        RADIO_CHANNEL         => 16#01#,
                        RADIO_DATARATE        => 16#02#,
                        RADIO_CONTWAVE        => 16#03#,
                        RADIO_RSSI            => 16#04#,
                        RADIO_ADDRESS         => 16#05#,
                        PM_SOURCE             => 16#10#,
                        PM_ONOFF_SWITCHOFF    => 16#11#,
                        PM_BATTERY_VOLTAGE    => 16#12#,
                        PM_BATTERY_STATE      => 16#13#,
                        PM_BATTERY_AUTOUPDATE => 16#14#,
                        OW_SCAN               => 16#20#,
                        OW_GETINFO            => 16#21#,
                        OW_READ               => 16#22#,
                        OW_WRITE              => 16#23#);
   for Packet_Type'Size use 8;

   --  Type for Syslink packet data.
   subtype Syslink_Data is Types.T_Uint8_Array (1 .. MTU);

   --  Type for Syslink packets.
   type Packet is record
      Slp_Type : Packet_Type;
      Length   : Types.T_Uint8;
      Data     : Syslink_Data;
   end record;

   --  Initialize the Syslink protocol.
   procedure Init;

   --  Test the Syslink protocol.
   function Test return Boolean;

   --  Send a packet to the nrf51 chip.
   procedure Send_Packet (Sl_Packet : Packet);

   --  Tasks and protected objects

   task type Task_Type (Prio : System.Priority) is
      pragma Priority (Prio);
   end Task_Type;

private

   --  Global variables and constants

   --  Synchronization bytes.
   START_BYTE1 : constant Types.T_Uint8 := 16#BC#;
   START_BYTE2 : constant Types.T_Uint8 := 16#CF#;

   --  Bitwise mask to get the group type of a packet.
   GROUP_MASK : constant Types.T_Uint8 := 16#F0#;

   --  Buffer used for transmission.
   Tx_Buffer : UART_Syslink.DMA_Data;

   Is_Init         : Boolean := False;
   Syslink_Access  : Ada.Synchronous_Task_Control.Suspension_Object;
   Dropped_Packets : Natural := 0;

   --  Types

   --  Syslink packet group type.
   type Packet_Group_Type is (RADIO_GROUP,
                              PM_GROUP,
                              OW_GROUP);

   for Packet_Group_Type use (RADIO_GROUP => 16#00#,
                              PM_GROUP    => 16#10#,
                              OW_GROUP    => 16#20#);
   for Packet_Group_Type'Size use 8;

   type Rx_States is (WAIT_FOR_FIRST_START,
                      WAIT_FOR_SECOND_START,
                      WAIT_FOR_TYPE,
                      WAIT_FOR_LENGTH,
                      WAIT_FOR_DATA,
                      WAIT_FOR_CHKSUM_1,
                      WAIT_FOR_CHKSUM_2);

   --  Procedures and functions

   function T_Uint8_To_Slp_Type is new Ada.Unchecked_Conversion
     (Types.T_Uint8, Packet_Type);

   --  Route the incoming packet by sending it to the appropriate layer
   --  (Radiolink, Power_Management etc.).
   procedure Route_Incoming_Packet (Rx_Sl_Packet : Packet);

end Syslink;
