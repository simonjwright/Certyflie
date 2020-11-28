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

with Link_Interface;
pragma Elaborate (Link_Interface);

with LEDS;

package body CRTP is

   ------------------
   -- Tx_Task_Type --
   ------------------

   task body Tx_Task_Type is
      Pkt         : Packet;
      Has_Succeed : Boolean;
      pragma Unreferenced (Has_Succeed);
   begin
      loop
         Tx_Queue.Await_Item_To_Dequeue
           (Pkt);

         Has_Succeed := Link_Interface.Send_Packet (Pkt);
      end loop;
   end Tx_Task_Type;

   ------------------
   -- Rx_Task_Type --
   ------------------

   task body Rx_Task_Type is
      Pkt         : Packet;
      Has_Succeed : Boolean;
   begin
      loop
         Link_Interface.Receive_Packet_Blocking (Pkt);

         if Callbacks (Pkt.Port) /= null then
            Callbacks (Pkt.Port) (Pkt);
         else
            Port_Queues (Pkt.Port).Enqueue_Item (Pkt, Has_Succeed);
         end if;
      end loop;
   end Rx_Task_Type;

   -------------------
   -- Create_Packet --
   -------------------

   function Create_Packet
     (Port    : Port_T;
      Channel : Channel_T) return Packet_Handler is
      Pkt : Packet;
      Handler : Packet_Handler;
   begin
      Pkt.Size := 0;
      Pkt.Reserved := 0;
      Pkt.Port := Port;
      Pkt.Channel := Channel;

      Handler.Index := 1;
      Handler.Pkt := Pkt;

      return Handler;
   end Create_Packet;

   -----------------------------
   -- Get_Handler_From_Packet --
   -----------------------------

   function Get_Handler_From_Packet
     (Pkt : Packet) return Packet_Handler
   is
      Handler : Packet_Handler;
   begin
      Handler.Pkt := Pkt;
      Handler.Index := Integer (Pkt.Size);

      return Handler;
   end Get_Handler_From_Packet;

   -----------------------------
   -- Get_Packet_From_Handler --
   -----------------------------

   function Get_Packet_From_Handler
     (Handler : Packet_Handler) return Packet is
   begin
      return Handler.Pkt;
   end Get_Packet_From_Handler;

   --------------
   -- Get_Data --
   --------------

   procedure Get_Data
     (Handler     : Packet_Handler;
      Index       : Positive;
      Data        : out T_Data)
   is
      Data_Size : constant Natural := (T_Data'Size + 7) / 8;
      subtype Byte_Array_Data is Types.T_Uint8_Array (1 .. Data_Size);

      function Byte_Array_To_Data is new Ada.Unchecked_Conversion
        (Byte_Array_Data, T_Data);
   begin
      if Index in Handler.Pkt.Data_1'First ..
        Handler.Pkt.Data_1'Last - Data_Size - 1
      then
         Data := Byte_Array_To_Data
           (Handler.Pkt.Data_1 (Index .. Index + Data_Size - 1));
      else
         raise Constraint_Error with "CRTP.Get_Data: Index out of range";
      end if;
   end Get_Data;

   -----------------
   -- Append_Data --
   -----------------

   procedure Append_Data
     (Handler : in out Packet_Handler;
      Data           : T_Data)
   is
      Data_Size : constant Natural := (T_Data'Size + 7) / 8;

      subtype Byte_Array_Data is Types.T_Uint8_Array (1 .. Data_Size);

      function Data_To_Byte_Array is new Ada.Unchecked_Conversion
        (T_Data, Byte_Array_Data);

      use type Types.T_Uint8;
   begin
      if Handler.Index + Data_Size - 1 <= Handler.Pkt.Data_1'Last then
         Handler.Pkt.Data_1
           (Handler.Index .. Handler.Index + Data_Size - 1) :=
           Data_To_Byte_Array (Data);

         Handler.Pkt.Size := Handler.Pkt.Size + Types.T_Uint8 (Data_Size);
         Handler.Index := Handler.Index + Data_Size;
      else
         raise Constraint_Error with "CRTP.Append_Data: Inde out of range";
      end if;
   end Append_Data;

   -------------------------
   -- Append_Data_If_Room --
   -------------------------

   procedure Append_Data_If_Room
     (Handler : in out Packet_Handler;
      Data    :        T_Data;
      Success :    out Boolean)
   is
      Data_Size : constant Natural := (T_Data'Size + 7) / 8;

      subtype Byte_Array_Data is Types.T_Uint8_Array (1 .. Data_Size);

      function Data_To_Byte_Array is new Ada.Unchecked_Conversion
        (T_Data, Byte_Array_Data);

      use type Types.T_Uint8;
   begin
      if Handler.Index + Data_Size - 1 <= Handler.Pkt.Data_1'Last then
         Handler.Pkt.Data_1
           (Handler.Index .. Handler.Index + Data_Size - 1) :=
             Data_To_Byte_Array (Data);

         Handler.Pkt.Size := Handler.Pkt.Size + Types.T_Uint8 (Data_Size);
         Handler.Index := Handler.Index + Data_Size;
         Success := True;
      else
         Success := False;
      end if;
   end Append_Data_If_Room;

   -------------------
   -- Reset_Handler --
   -------------------

   procedure Reset_Handler (Handler : in out Packet_Handler) is
   begin
      Handler.Index := 1;
      Handler.Pkt.Size := 0;
      Handler.Pkt.Data_1 := (others => 0);
   end Reset_Handler;

   ---------------------
   -- Get_Packet_Size --
   ---------------------

   function Get_Packet_Size
     (Handler : Packet_Handler) return Types.T_Uint8 is
   begin
      return Handler.Pkt.Size;
   end Get_Packet_Size;

   -----------------------------
   -- Receive_Packet_Blocking --
   -----------------------------

   procedure Receive_Packet_Blocking
     (Pkt     : out Packet;
      Port_ID : Port_T) is
   begin
      Port_Queues (Port_ID).Await_Item_To_Dequeue (Pkt);
   end Receive_Packet_Blocking;

   -----------------
   -- Send_Packet --
   -----------------

   procedure Send_Packet
     (Pkt          :     Packet;
      Has_Succeed  : out Boolean;
      Time_To_Wait :     Ada.Real_Time.Time_Span
        := Ada.Real_Time.Time_Span_Zero)
   is
      pragma Unreferenced (Time_To_Wait);
   begin
      Tx_Queue.Enqueue_Item (Pkt, Has_Succeed);
   end Send_Packet;

   -----------------------
   -- Register_Callback --
   -----------------------

   procedure Register_Callback
        (Port_ID : Port_T;
         Callbk  : Callback) is
   begin
      Callbacks (Port_ID) := Callbk;
   end Register_Callback;

   -------------------------
   -- Unregister_Callback --
   -------------------------

   procedure Unregister_Callback (Port_ID : Port_T) is
   begin
      Callbacks (Port_ID) := null;
   end Unregister_Callback;

   -----------
   -- Reset --
   -----------

   procedure Reset is
   begin
      Tx_Queue.Reset_Queue;
      --  TODO: reset the link queues too.
   end Reset;

   ----------------------
   -- Set_Is_Connected --
   ----------------------

   procedure Set_Is_Connected (Value : Boolean) is
   begin
      Connected := Value;
      LEDS.Set_Link_State ((if Value
                            then LEDS.Connected
                            else LEDS.Not_Connected));
   end Set_Is_Connected;

   ------------------
   -- Is_Connected --
   ------------------

   function Is_Connected return Boolean is
   begin
      --  This is what crazyflie-firmware/src/modules/src/crtp.c does
      return True;
      --  return Connected;
   end Is_Connected;

end CRTP;
