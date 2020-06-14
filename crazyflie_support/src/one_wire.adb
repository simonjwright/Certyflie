------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--        Copyright (C) 2020, Simon Wright <simon@pushface.org>             --
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

package body One_Wire is

   Unimplemented : exception;

   Read_Size : constant := 29;
   Max_Write_Size : constant := 26;

   Is_Init         : Boolean := False;

   --  The data transferred to/from Open Wire in the Data component of
   --  Syslink packets.
   --  The first byte corresponds to the nmem field in owCommand_s in
   --  src/hal/interface/ow.h.
   type OW_Comms_Type is (Raw, Scan, Getinfo, Read, Write);
   type OW_Comms_Data (Kind : OW_Comms_Type) is record
      case Kind is
         when Raw =>
            Data : Types.T_Uint8_Array (1 .. 31);
         when Scan =>
            Scan_Number_Of_Targets : Types.T_Uint8;
         when Getinfo =>
            Info_Target : Types.T_Uint8;
            Info_Target_ID : Types.T_Uint8_Array (1 .. 8);
         when Read =>
            Read_Target : Types.T_Uint8;
            Read_Address : Types.T_Uint16;
            Read_Data : Types.T_Uint8_Array (1 .. Read_Size);
         when Write =>
            Write_Target : Types.T_Uint8;
            Write_Address : Types.T_Uint16;
            Length : Types.T_Uint16;
            Write_Data : Types.T_Uint8_Array (1 .. Max_Write_Size);
      end case;
   end record with Unchecked_Union, Pack, Size => 32 * 8;

   protected Response_Buffer is
      entry Get (Response : out Syslink.Packet);
      procedure Put (Response : Syslink.Packet);
   private
      Available : Boolean := False;
      Buffered_Response : Syslink.Packet;
   end Response_Buffer;

   procedure Transfer (Packet_Type    :     OW_Packet_Type;
                       Data           :     OW_Comms_Data;
                       Length_To_Send :     Natural;
                       Response       : out OW_Comms_Data)
   with Pre => Length_To_Send <= 31;

   procedure Init is
   begin
      Syslink.Init;
      --  ?
      Is_Init := True;
   end Init;

   function Test return Boolean is (Is_Init);

   function Scan (Number_Of_Targets : out Types.T_Uint8) return Boolean
   is
      Response : OW_Comms_Data (Kind => Scan);
      pragma Warnings (Off, "*may be referenced before it has a value*");
   begin
      Transfer (Packet_Type    => Syslink.OW_SCAN,
                Data           => Response,
                Length_To_Send => 0,
                Response       => Response);
      pragma Warnings (On, "*may be referenced before it has a value*");
      Number_Of_Targets := Response.Scan_Number_Of_Targets;
      return True;
   end Scan;

   function Get_Info (Target : Types.T_Uint8;
                      Serial : out Target_Serial) return Boolean
   is
      Response : OW_Comms_Data (Kind => Getinfo);
   begin
      Response.Info_Target := Target;
      Transfer (Packet_Type    => Syslink.OW_GETINFO,
                Data           => Response,
                Length_To_Send => 1,
                Response       => Response);
      Serial := Response.Info_Target_ID;
      return True;
   end Get_Info;

   function Read (Target  :     Types.T_Uint8;
                  Address :     Types.T_Uint16;
                  Length  :     Types.T_Uint8;
                  Data    : out Types.T_Uint8_Array) return Boolean
   is
      pragma Unreferenced (Target);
      pragma Unreferenced (Address);
      pragma Unreferenced (Length);
      pragma Unreferenced (Data);
   begin
      return raise Unimplemented;
   end Read;

   function Write (Target  : Types.T_Uint8;
                   Address : Types.T_Uint16;
                   Length  : Types.T_Uint8;
                   Data    : Types.T_Uint8_Array) return Boolean
   is
      pragma Unreferenced (Target);
      pragma Unreferenced (Address);
      pragma Unreferenced (Length);
      pragma Unreferenced (Data);
   begin
      return raise Unimplemented;
   end Write;

   protected body Response_Buffer is
      entry Get (Response : out Syslink.Packet) when Available is
      begin
         Response := Buffered_Response;
         Available := False;
      end Get;

      procedure Put (Response : Syslink.Packet) is
         pragma Assert (not Available, "unread response waiting");
      begin
         Buffered_Response := Response;
         Available := True;
      end Put;
   end Response_Buffer;

   procedure Transfer (Packet_Type    :     OW_Packet_Type;
                       Data           :     OW_Comms_Data;
                       Length_To_Send :     Natural;
                       Response       : out OW_Comms_Data)
   is
      Pkt : Syslink.Packet;
   begin
      Pkt.Slp_Type := Packet_Type;
      Pkt.Length := Types.T_Uint8 (Length_To_Send);
      Pkt.Data (1 .. Length_To_Send) := Data.Data (1 .. Length_To_Send);
      Syslink.Send_Packet (Pkt);
      Response_Buffer.Get (Pkt);
      Response.Data := Pkt.Data;
   end Transfer;

   procedure Receive (Pkt : Syslink.Packet)
      --  Routed to by Syslink on reception.
   is
   begin
      Response_Buffer.Put (Pkt);
   end Receive;

end One_Wire;
