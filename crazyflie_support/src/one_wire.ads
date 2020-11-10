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

with Syslink;
with Types;

package One_Wire is

   subtype OW_Packet_Type
     is Syslink.Packet_Type range Syslink.OW_SCAN .. Syslink.OW_WRITE;

   --  See src/hal/interface/ow.h
   Max_Transfer_Size : constant := 112;

   subtype Target_Serial is Types.T_Uint8_Array (1 .. 8);

   procedure Init;

   function Test return Boolean;

   function Scan (Number_Of_Targets : out Types.T_Uint8) return Boolean;

   function Get_Info (Target : Types.T_Uint8;
                      Serial : out Target_Serial) return Boolean;

   function Read (Target  :     Types.T_Uint8;
                  Address :     Types.T_Uint16;
                  Length  :     Types.T_Uint8;
                  Data    : out Types.T_Uint8_Array) return Boolean
   with Pre => Data'Length <= Max_Transfer_Size;

   function Write (Target  : Types.T_Uint8;
                   Address : Types.T_Uint16;
                   Length  : Types.T_Uint8;
                   Data    : Types.T_Uint8_Array) return Boolean
   with Pre => Data'Length <= Max_Transfer_Size;

end One_Wire;
