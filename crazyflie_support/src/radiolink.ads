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
   procedure Send_Packet (Packet : CRTP.Packet;
                          One_Off : Boolean := False);

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

   --  This queue gives preference to one-off packets, which we don't
   --  want to drop.
   protected Tx_Queue is
      --  If Enqueue_One_Off_Item fails, the caller can choose to
      --  delay before trying again.
      procedure Enqueue_One_Off_Item (Item    : Syslink.Packet;
                                      Success : out Boolean);

      --  Enqueue_Continuous_Item will just drop the itm if it can't
      --  be queued.
      procedure Enqueue_Continuous_Item (Item : Syslink.Packet);

      procedure Dequeue_Item (Item    : out Syslink.Packet;
                              Success : out Boolean);

      procedure Reset_Queue;
   private
      One_Off_Queue    : Syslink_Queue.T_Queue (TX_QUEUE_SIZE / 2);
      Continuous_Queue : Syslink_Queue.T_Queue (TX_QUEUE_SIZE / 2);
   end Tx_Queue;

   pragma Warnings (Off,  "violate restriction No_Implicit_Heap_Allocation");

   --  --  Protected object queue used for transmission of messages that
   --  --  shouldn't be dropped.
   --  One_Off_Queue : Syslink_Queue.Protected_Queue
   --    (System.Interrupt_Priority'Last, TX_QUEUE_SIZE / 2);

   --  --  Protected object queue used for transmission of messages that
   --  --  can be dropped without problems.
   --  Continuous_Queue : Syslink_Queue.Protected_Queue
   --    (System.Interrupt_Priority'Last, TX_QUEUE_SIZE / 2);

   --  Protected object queue used for reception.
   Rx_Queue : CRTP_Queue.Protected_Queue
     (System.Interrupt_Priority'Last, RX_QUEUE_SIZE);

   pragma Warnings (On,  "violate restriction No_Implicit_Heap_Allocation");

end Radiolink;
