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

with CRTP;
with Types;

private with System;
private with Syslink;
private with Generic_Queue;

package Radiolink is

   --  Procedures and functions

   --  Initialize the Radiolink layer.
   procedure Init;

   --  Set the radio channel.
   procedure Set_Channel (Channel : Types.T_Uint8);

   --  Set the radio address.
   procedure Set_Address (Address : Types.T_Uint64);

   --  Set the radio data rate.
   procedure Set_Data_Rate (Data_Rate : Types.T_Uint8);

   --  Send a packet to Radiolink layer.
   function Send_Packet (Packet : CRTP.Packet) return Boolean;

   --  Receive a packet from Radiolink layer.
   --  Putting the task calling it in sleep mode until a packet is received.
   procedure Receive_Packet_Blocking (Packet : out CRTP.Packet);

private

   --  Size of transmission/receptions queues.
   TX_QUEUE_SIZE : constant := 60;
   RX_QUEUE_SIZE : constant := 60;

   package Syslink_Queue is new Generic_Queue (Syslink.Packet);
   package CRTP_Queue is new Generic_Queue (CRTP.Packet);

   --  Global variables and constants

   Is_Init : Boolean := False;
   RSSI    : Types.T_Uint8;

   --  Tasks and protected objects

   pragma Warnings (Off,  "violate restriction No_Implicit_Heap_Allocation");

   --  Protected object queue used for transmission.
   Tx_Queue : Syslink_Queue.Protected_Queue
     (System.Interrupt_Priority'Last, TX_QUEUE_SIZE);

   --  Protected object queue used for reception.
   Rx_Queue : CRTP_Queue.Protected_Queue
     (System.Interrupt_Priority'Last, RX_QUEUE_SIZE);

   pragma Warnings (On,  "violate restriction No_Implicit_Heap_Allocation");

end Radiolink;
