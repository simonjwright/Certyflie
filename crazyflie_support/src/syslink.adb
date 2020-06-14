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

with One_Wire;
with Power_Management;
with Radiolink;

package body Syslink is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      UART_Syslink.Init;

      Ada.Synchronous_Task_Control.Set_True (Syslink_Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   -----------------
   -- Send_Packet --
   -----------------

   procedure Send_Packet (Sl_Packet : Packet)
   is
      Data_Size : Natural;
      Chk_Sum   : array (1 .. 2) of Types.T_Uint8 := (others => 0);
      use type Types.T_Uint8;
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (Syslink_Access);

      Tx_Buffer (1) := START_BYTE1;
      Tx_Buffer (2) := START_BYTE2;
      Tx_Buffer (3) := Packet_Type'Enum_Rep (Sl_Packet.Slp_Type);
      Tx_Buffer (4) := Sl_Packet.Length;

      Data_Size := Natural (Sl_Packet.Length) + 6;
      Tx_Buffer (5 .. Data_Size - 2) :=
        Sl_Packet.Data (1 .. Integer (Sl_Packet.Length));

      for I in 3 .. Data_Size - 2 loop
         Chk_Sum (1) := Chk_Sum (1) + Tx_Buffer (I);
         Chk_Sum (2) := Chk_Sum (2) + Chk_Sum (1);
      end loop;

      Tx_Buffer (Data_Size - 1) := Chk_Sum (1);
      Tx_Buffer (Data_Size) := Chk_Sum (2);

      UART_Syslink.Send_DMA_Data_Blocking (Data_Size, Tx_Buffer);
      Ada.Synchronous_Task_Control.Set_True (Syslink_Access);
   end Send_Packet;

   ---------------------------
   -- Route_Incoming_Packet --
   ---------------------------

   procedure Route_Incoming_Packet (Rx_Sl_Packet : Packet)
   is
      Group_Type_Raw : Types.T_Uint8;
      Group_Type     : Packet_Group_Type;

      use type Types.T_Uint8;
   begin
      Group_Type_Raw := Packet_Type'Enum_Rep (Rx_Sl_Packet.Slp_Type)
        and GROUP_MASK;
      Group_Type := Packet_Group_Type'Enum_Val (Group_Type_Raw);

      case Group_Type is
         when RADIO_GROUP =>
            Radiolink.Syslink_Dispatch (Rx_Sl_Packet);
         when PM_GROUP =>
            Power_Management.Syslink_Update (Rx_Sl_Packet);
         when OW_GROUP =>
            One_Wire.Receive (Rx_Sl_Packet);
      end case;
   end Route_Incoming_Packet;

   ---------------
   -- Task_Type --
   ---------------

   task body Task_Type is
      Rx_State     : Rx_States := WAIT_FOR_FIRST_START;
      Rx_Sl_Packet : Packet;
      Rx_Byte      : Types.T_Uint8;
      Data_Index   : Positive := 1;
      Chk_Sum      : array (1 .. 2) of Types.T_Uint8;
      use type Types.T_Uint8;
   begin
      loop
         UART_Syslink.Get_Data_Blocking (Rx_Byte);

         case Rx_State is
            when WAIT_FOR_FIRST_START =>

               Rx_State := (if Rx_Byte = START_BYTE1 then
                               WAIT_FOR_SECOND_START
                            else
                               WAIT_FOR_FIRST_START);
            when WAIT_FOR_SECOND_START =>
               Rx_State := (if Rx_Byte = START_BYTE2 then
                               WAIT_FOR_TYPE
                            else
                               WAIT_FOR_FIRST_START);
            when WAIT_FOR_TYPE =>
               Chk_Sum (1) := Rx_Byte;
               Chk_Sum (2) := Rx_Byte;
               Rx_Sl_Packet.Slp_Type := T_Uint8_To_Slp_Type (Rx_Byte);
               Rx_State := WAIT_FOR_LENGTH;
            when WAIT_FOR_LENGTH =>
               if Rx_Byte <= MTU then
                  Rx_Sl_Packet.Length := Rx_Byte;
                  Chk_Sum (1) := Chk_Sum (1) + Rx_Byte;
                  Chk_Sum (2) := Chk_Sum (2) + Chk_Sum (1);
                  Data_Index := 1;
                  Rx_State := (if Rx_Byte > 0 then
                                  WAIT_FOR_DATA
                               else
                                  WAIT_FOR_CHKSUM_1);
               else
                  Rx_State := WAIT_FOR_FIRST_START;
               end if;
            when WAIT_FOR_DATA =>
               Rx_Sl_Packet.Data (Data_Index) := Rx_Byte;
               Chk_Sum (1) := Chk_Sum (1) + Rx_Byte;
               Chk_Sum (2) := Chk_Sum (2) + Chk_Sum (1);
               Data_Index := Data_Index + 1;
               if Types.T_Uint8 (Data_Index) > Rx_Sl_Packet.Length then
                  Rx_State := WAIT_FOR_CHKSUM_1;
               end if;
            when WAIT_FOR_CHKSUM_1 =>
               if Chk_Sum (1) = Rx_Byte then
                  Rx_State := WAIT_FOR_CHKSUM_2;
               else
                  Dropped_Packets := Dropped_Packets + 1;
                  Rx_State := WAIT_FOR_FIRST_START;
               end if;
            when WAIT_FOR_CHKSUM_2 =>
               if Chk_Sum (2) = Rx_Byte then
                  Route_Incoming_Packet (Rx_Sl_Packet);
               else
                  Dropped_Packets := Dropped_Packets + 1;
               end if;
               Rx_State := WAIT_FOR_FIRST_START;
         end case;
      end loop;
   end Task_Type;

end Syslink;
