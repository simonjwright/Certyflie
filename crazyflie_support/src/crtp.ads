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

with System;
with Ada.Real_Time;

with Generic_Queue;
with Types;

package CRTP is

   --  Constants

   MAX_DATA_SIZE : constant := 30;
   TX_QUEUE_SIZE : constant := 60;
   RX_QUEUE_SIZE : constant := 2;
   NBR_OF_PORTS  : constant := 16;

   --  Types

   --  Type used for representing a CRTP channel, which can be seen
   --  as a sub-set for a specific port.
   type Channel_T is new Types.T_Uint8 range 0 .. 3;
   for Channel_T'Size use 2;

   --  Enumeration type for CRTP ports. Each port corresponds to
   --  a specific module.
   type Port_T is (PORT_CONSOLE,
                   PORT_PARAM,
                   PORT_SETPOINT,
                   PORT_MEM,
                   PORT_LOG,
                   PORT_LOCALIZATION,
                   PORT_SETPOINT_GENERIC,
                   PORT_SETPOINT_HL,
                   PORT_PLATFORM,
                   PORT_LINK);
   for Port_T use (PORT_CONSOLE          => 16#00#,
                   PORT_PARAM            => 16#02#,
                   PORT_SETPOINT         => 16#03#,
                   PORT_MEM              => 16#04#,
                   PORT_LOG              => 16#05#,
                   PORT_LOCALIZATION     => 16#06#,
                   PORT_SETPOINT_GENERIC => 16#07#,
                   PORT_SETPOINT_HL      => 16#08#,
                   PORT_PLATFORM         => 16#0D#,
                   PORT_LINK             => 16#0F#);
   for Port_T'Size use 4;

   --  Type for representing the two reserved bits in a CRTP packet.
   --  These bits are used for the transport layer.
   type Reserved_T is new Types.T_Uint8 range 0 .. 3;
   for Reserved_T'Size use 2;

   --  Type for CRTP packet data.
   subtype Data_T is Types.T_Uint8_Array (1 .. MAX_DATA_SIZE);

   --  Type used to represenet a raw CRTP Packet (Header + Data).
   subtype Raw_T is Types.T_Uint8_Array (1 .. MAX_DATA_SIZE + 1);

   --  Type listing the different representations for the union type
   --  CRTP Packet.
   type Packet_Representation is (DETAILED, HEADER_DATA, RAW_Packet);

   --  Type for CRTP packets.
   type Packet (Repr : Packet_Representation := DETAILED) is record
      Size     : Types.T_Uint8;

      case Repr is
         when DETAILED =>
            Channel  : Channel_T;
            Reserved : Reserved_T;
            Port     : Port_T;
            Data_1   : Data_T;
         when HEADER_DATA =>
            Header : Types.T_Uint8;
            Data_2 : Data_T;
         when RAW_Packet =>
            Raw_Pkt : Raw_T;
      end case;
   end record;

   pragma Unchecked_Union (Packet);
   for Packet'Size use 256;
   pragma Pack (Packet);

   --  Type used to easily manipulate CRTP packet.
   type Packet_Handler is private;

   --  Procedures and functions

   --  Create a CRTP Packet handler to append/get data.
   function Create_Packet
     (Port    : Port_T;
      Channel : Channel_T) return Packet_Handler;

   --  Return a handler to easily manipulate the CRTP packet.
   function Get_Handler_From_Packet
     (Pkt : Packet) return Packet_Handler;

   --  Return the CRTP Packet contained in the CRTP Packet handler.
   function Get_Packet_From_Handler
     (Handler : Packet_Handler) return Packet;

   --  Append data to the CRTP Packet. Raises CE if there's not enough room.
   generic
      type T_Data is private;
   procedure Append_Data
     (Handler        : in out Packet_Handler;
      Data           : T_Data);

   --  Append data to the CRTP Packet. If 'Success' is returned False,
   --  the data wasn't appended.
   generic
      type T_Data is private;
   procedure Append_Data_If_Room
     (Handler : in out Packet_Handler;
      Data    :        T_Data;
      Success :    out Boolean);

   --  Get data at a specified index of the CRTP Packet data field.
   generic
      type T_Data is private;
   procedure Get_Data
     (Handler     : Packet_Handler;
      Index       : Positive;
      Data        : out T_Data);

   --  Function pointer type for callbacks.
   type Callback is access procedure (Pkt : Packet);

   --  Reset the index, the size and the data contained in the handler.
   procedure Reset_Handler (Handler : in out Packet_Handler);

   --  Get the size of the CRTP packet contained in the handler.
   function Get_Packet_Size
     (Handler : Packet_Handler) return Types.T_Uint8;

   --  Receive a packet from the port queue, putting the task calling it
   --  in sleep mode while a packet has not been received
   procedure Receive_Packet_Blocking
     (Pkt     : out Packet;
      Port_ID :     Port_T);

   --  Send a packet, with a given Timeout
   procedure Send_Packet
     (Pkt          :     Packet;
      Has_Succeed  : out Boolean;
      Time_To_Wait :     Ada.Real_Time.Time_Span
        := Ada.Real_Time.Time_Span_Zero);

   --  Register a callback to be called when a packet is received in
   --  the port queue
   procedure Register_Callback
     (Port_ID : Port_T;
      Callbk  : Callback);

   --  Unregister the callback for this port.
   procedure Unregister_Callback (Port_ID : Port_T);

   --  Reset the CRTP module by flushing the Tx Queue.
   procedure Reset;

   --  Used by the Commander module to state if we are still connected or not.
   procedure Set_Is_Connected (Value : Boolean);

   --  Used to know if we are still connected.
   function Is_Connected return Boolean;

   --  Task in charge of transmitting the messages in the Tx Queue
   --  to the link layer.
   task type Tx_Task_Type (Prio : System.Priority) is
      pragma Priority (Prio);
   end Tx_Task_Type;

   --  Task in charge of dequeuing the messages in the Rx_queue
   --  to put them in the Port_Queues.
   task type Rx_Task_Type (Prio : System.Priority) is
      pragma Priority (Prio);
   end Rx_Task_Type;

private
   package Queue is new Generic_Queue (Packet);

   --  Types
   type Packet_Handler is record
      Pkt   : Packet;
      Index : Positive;
   end record;

   --  Tasks and protected objects

   pragma Warnings (Off,  "violate restriction No_Implicit_Heap_Allocation");
   --  Protected object queue for transmission.
   Tx_Queue : Queue.Protected_Queue
     (System.Interrupt_Priority'Last, TX_QUEUE_SIZE);

   --  Protected object queue for reception.
   Rx_Queue : Queue.Protected_Queue
     (System.Interrupt_Priority'Last, RX_QUEUE_SIZE);
   pragma Warnings (On,  "violate restriction No_Implicit_Heap_Allocation");

   --  Array of protected object queues, one for each task.
   Port_Queues : array (Port_T) of Queue.Protected_Queue
     (System.Interrupt_Priority'Last, 1);

   --  Array of callbacks when a packet is received.
   Callbacks : array (Port_T) of Callback := (others => null);

   --  Global variables

   --  Number of dropped packets.
   Dropped_Packets : Natural := 0;

   --  Used to know if we are still connected or not.
   Connected : Boolean := False;

end CRTP;
