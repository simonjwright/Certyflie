------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
--        Copyright (C) 2017-2020, Simon Wright <simon@pushface.org>        --
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

with CRC;
with CRTP;
with Types;

package body Parameter is

   --  Types, subprograms previously in private part of spec

   --  Type representing all the available parameter module CRTP channels.
   type Channel is
     (TOC_CH,
      READ_CH,
      WRITE_CH,
      MISC_CH);
   for Channel use
     (TOC_CH   => 0,
      READ_CH  => 1,
      WRITE_CH => 2,
      MISC_CH  => 3);
   for Channel'Size use 2;

   --  Type representing all the param commands.
   --  GET_INFO is requested at connexion to fetch the TOC.
   --  GET_ITEM is requested whenever the client wants to
   --  fetch the newest variable data.
   type TOC_Command is
     (GET_ITEM,
      GET_INFO);
   for TOC_Command use
     (GET_ITEM => 0,
      GET_INFO => 1);
   for TOC_Command'Size use 8;

   --  Type representing all the available parameter control commands.
   --  Doesn't seem to be used? Not even in the C!
   type Control_Command is
     (RESET,
      GET_NEXT,
      GET_CRC);
   for Control_Command use
     (RESET    => 0,
      GET_NEXT => 1,
      GET_CRC  => 2);
   for Control_Command'Size use 8;

   --  Error code constants
   ENOENT : constant := 2;
   --  E2BIG  : constant := 7;
   --  ENOMEM : constant := 12;
   --  EEXIST : constant := 17;

   --  Maximum number of groups we can log.
   MAX_NUMBER_OF_GROUPS    : constant := 8;
   --  Maximum number of variables we can log inside a group.
   MAX_VARIABLES_PER_GROUP : constant := 16;
   pragma Assert
     (MAX_NUMBER_OF_GROUPS * MAX_VARIABLES_PER_GROUP
        <= Integer (Types.T_Uint8'Last));

   --  XXX This opaque design may require all the variables in one
   --  group to be created together (no interpolation allowed).

   --  Types, subprograms previously in private part of spec

   subtype Parameter_Name is String (1 .. MAX_VARIABLE_NAME_LENGTH);

   --  Type representing a parameter variable.
   type Parameter_Variable is record
      Group_ID       : Natural;
      Name           : Parameter_Name;
      Name_Length    : Natural;
      Parameter_Type : Parameter_Variable_Type;
      Variable       : System.Address := System.Null_Address;
   end record;

   type Parameter_Group_Variable_Array is
     array (0 .. MAX_VARIABLES_PER_GROUP - 1) of
     aliased Parameter_Variable;

   type Parameter_Variable_Array is
     array (0 .. MAX_VARIABLES_PER_GROUP * MAX_NUMBER_OF_GROUPS - 1)
     of access Parameter_Variable;

   --  Type representing a log group.
   --  Parameter groups can contain several log variables.
   type Parameter_Group is record
      Name            : Parameter_Name;
      Name_Length     : Natural;
      Variables       : Parameter_Group_Variable_Array;
      Variables_Count : Natural := 0;
   end record;

   type Parameter_Group_Array is
     array (0 .. MAX_NUMBER_OF_GROUPS - 1) of Parameter_Group;

   type Parameter_Database is record
      Groups          : Parameter_Group_Array;
      Variables       : Parameter_Variable_Array := (others => null);
      Groups_Count    : Natural := 0;
      Variables_Count : Natural := 0;
   end record;

   --  Global variables and constants

   Is_Init : Boolean := False;

   --  Head of the parameter groups list.
   Parameters : aliased Parameter_Database;

   --  Procedures and functions

   --  Handler called when a CRTP packet is received in the param
   --  port.
   procedure CRTP_Handler (Packet : CRTP.Packet);

   --  Process a command related to TOC demands from the python client.
   procedure TOC_Process (Packet : CRTP.Packet);

   --  Convert an unbounded string to a Log_Name, with a fixed size.
   function String_To_Name (Name : String) return Parameter_Name;
   pragma Inline (String_To_Name);

   --  Append raw data from the variable and group name.
   procedure Append_Raw_Data_Variable_Name_To_Packet
     (Variable       : Parameter_Variable;
      Group          : Parameter_Group;
      Packet_Handler : in out CRTP.Packet_Handler);

   --  Read a parameter.
   procedure Read_Process (Packet : CRTP.Packet);

   --  Write a parameter.
   procedure Write_Process (Packet : CRTP.Packet);

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      CRTP.Register_Callback (CRTP.PORT_PARAM, CRTP_Handler'Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   ----------------------------
   -- Create_Parameter_Group --
   ----------------------------

   procedure Create_Parameter_Group
     (Name     :     String;
      Group_ID : out Natural)
   is
      Groups_Count : constant Natural := Parameters.Groups_Count;
   begin
      if Groups_Count > Parameters.Groups'Last then
         raise Constraint_Error with "too many parameter groups";
      end if;
      if Name'Length > MAX_VARIABLE_NAME_LENGTH then
         raise Constraint_Error with "parameter group name too long";
      end if;

      Parameters.Groups (Groups_Count) :=
        (Name        => String_To_Name (Name),
         Name_Length => Name'Length,
         others      => <>);

      Group_ID                         := Groups_Count;
      Parameters.Groups_Count          := Groups_Count + 1;
   end Create_Parameter_Group;

   ----------------------------------------
   -- Append_Parameter_Variable_To_Group --
   ----------------------------------------

   procedure Append_Parameter_Variable_To_Group
     (Group_ID       : Natural;
      Name           : String;
      Parameter_Type : Parameter_Variable_Type;
      Variable       : System.Address)
   is
   begin
      if Group_ID > Parameters.Groups_Count then
         raise Constraint_Error with "unknown group ID" & Group_ID'Image;
      end if;

      if Name'Length > MAX_VARIABLE_NAME_LENGTH then
         raise Constraint_Error with "variable name too long";
      end if;

      declare
         Group : Parameter_Group renames  Parameters.Groups (Group_ID);
         Variables_Count : constant Natural := Group.Variables_Count;
         pragma Assert (Variables_Count < MAX_VARIABLES_PER_GROUP,
                        "more than"
                          & MAX_VARIABLES_PER_GROUP'Image
                          & " in group");
      begin
         Group.Variables (Variables_Count) :=
           (Group_ID       => Group_ID,
            Name           => String_To_Name (Name),
            Name_Length    => Name'Length,
            Parameter_Type => Parameter_Type,
            Variable       => Variable);
         Group.Variables_Count := Variables_Count + 1;

         Parameters.Variables (Parameters.Variables_Count)
           := Group.Variables (Variables_Count)'Access;
         Parameters.Variables_Count := Parameters.Variables_Count + 1;
      end;
   end Append_Parameter_Variable_To_Group;

   --  Private procedures and functions

   ------------------
   -- CRTP_Handler --
   ------------------

   procedure CRTP_Handler (Packet : CRTP.Packet)
   is
      Ch : constant Channel := Channel'Val (Packet.Channel);
   begin
      case Ch is
         when TOC_CH =>
            TOC_Process (Packet);
         when READ_CH =>
            Read_Process (Packet);
         when WRITE_CH =>
            Write_Process (Packet);
         when MISC_CH =>
            null;
      end case;
   end CRTP_Handler;

   -----------------
   -- TOC_Process --
   -----------------

   procedure TOC_Process (Packet : CRTP.Packet)
   is
      function T_Uint8_To_TOC_Command is new Ada.Unchecked_Conversion
        (Types.T_Uint8, TOC_Command);

      procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
        (Types.T_Uint8);

      procedure CRTP_Append_Parameter_Variable_Type_Data
      is new CRTP.Append_Data
        (Parameter_Variable_Type);

      procedure CRTP_Append_T_Uint32_Data is new CRTP.Append_Data
        (Types.T_Uint32);

      function Database_CRC32 return Types.T_Uint32;
      function Database_CRC32 return Types.T_Uint32 is
         --  Note, this doesn't take account of uninitialized
         --  components of Parameters, but since the only use (in
         --  cfclient, anyway) is to determine whether to load new
         --  data this will just add a minor startup load.
         function TOC_CRC
         is new CRC.Make (Data_Kind => Parameter_Database);
      begin
         return Types.T_Uint32 (TOC_CRC (Parameters));
      end Database_CRC32;

      Command        : TOC_Command;
      Packet_Handler : CRTP.Packet_Handler;
      Dummy          : Boolean;
   begin
      Command := T_Uint8_To_TOC_Command (Packet.Data_1 (1));
      Packet_Handler := CRTP.Create_Packet
        (CRTP.PORT_PARAM, Channel'Enum_Rep (TOC_CH));
      CRTP_Append_T_Uint8_Data
        (Packet_Handler,
         TOC_Command'Enum_Rep (Command));

      case Command is
         when GET_INFO =>
            CRTP_Append_T_Uint8_Data
              (Packet_Handler,
               Types.T_Uint8 (Parameters.Variables_Count));
            CRTP_Append_T_Uint32_Data
              (Packet_Handler, Database_CRC32);

         when GET_ITEM =>
            declare
               Var_ID              : constant Types.T_Uint8
                 := Packet.Data_1 (2);
               Parameter_Var       : Parameter_Variable;
               Parameter_Var_Group : Parameter_Group;
            begin
               if Natural (Var_ID) < Parameters.Variables_Count
               then
                  CRTP_Append_T_Uint8_Data
                    (Packet_Handler, Var_ID);

                  Parameter_Var := Parameters.Variables
                    (Integer (Var_ID)).all;
                  Parameter_Var_Group := Parameters.Groups
                    (Parameter_Var.Group_ID);

                  CRTP_Append_Parameter_Variable_Type_Data
                    (Packet_Handler,
                     Parameter_Var.Parameter_Type);
                  Append_Raw_Data_Variable_Name_To_Packet
                    (Parameter_Var,
                     Parameter_Var_Group,
                     Packet_Handler);
               end if;
            end;
      end case;
      CRTP.Send_Packet
        (CRTP.Get_Packet_From_Handler (Packet_Handler),
         Dummy);
   end TOC_Process;

   --------------------
   -- String_To_Name --
   --------------------

   function String_To_Name (Name : String) return Parameter_Name
   is
      Result : Parameter_Name := (others => ASCII.NUL);
   begin
      Result (1 .. Name'Length) := Name;

      return Result;
   end String_To_Name;

   ---------------------------------------------
   -- Append_Raw_Data_Variable_Name_To_Packet --
   ---------------------------------------------

   procedure Append_Raw_Data_Variable_Name_To_Packet
     (Variable       : Parameter_Variable;
      Group          : Parameter_Group;
      Packet_Handler : in out CRTP.Packet_Handler)
   is
      subtype Parameter_Complete_Name is
        String (1 .. Variable.Name_Length + Group.Name_Length + 2); -- nulls
      subtype Parameter_Complete_Name_Raw is
        Types.T_Uint8_Array (Parameter_Complete_Name'Range);

      ------------------------------------------------------------
      -- Parameter_Complete_Name_To_Parameter_Complete_Name_Raw --
      ------------------------------------------------------------

      function Parameter_Complete_Name_To_Parameter_Complete_Name_Raw is new
        Ada.Unchecked_Conversion (Parameter_Complete_Name,
                                  Parameter_Complete_Name_Raw);

      --------------------------------------------------
      -- CRTP_Append_Parameter_Complete_Name_Raw_Data --
      --------------------------------------------------

      procedure CRTP_Append_Parameter_Complete_Name_Raw_Data is new
        CRTP.Append_Data (Parameter_Complete_Name_Raw);

      Complete_Name : constant Parameter_Complete_Name
        := Group.Name (1 .. Group.Name_Length) & ASCII.NUL
        & Variable.Name (1 .. Variable.Name_Length) & ASCII.NUL;
      Complete_Name_Raw : Parameter_Complete_Name_Raw;
   begin
      Complete_Name_Raw :=
        Parameter_Complete_Name_To_Parameter_Complete_Name_Raw (Complete_Name);
      CRTP_Append_Parameter_Complete_Name_Raw_Data
        (Packet_Handler, Complete_Name_Raw);
   end Append_Raw_Data_Variable_Name_To_Packet;

   ------------------
   -- Read_Process --
   ------------------

   procedure Read_Process (Packet : CRTP.Packet) is
      procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
        (Types.T_Uint8);

      ID             : constant Types.T_Uint8 := Packet.Data_1 (1);
      Packet_Handler : CRTP.Packet_Handler;
   begin
      Packet_Handler := CRTP.Create_Packet
        (CRTP.PORT_PARAM, Channel'Enum_Rep (READ_CH));
      if Natural (ID) >= Parameters.Variables_Count then
         --  Invalid
         CRTP_Append_T_Uint8_Data (Packet_Handler, Types.T_Uint8'Last);
         CRTP_Append_T_Uint8_Data (Packet_Handler, ID);
         CRTP_Append_T_Uint8_Data (Packet_Handler, ENOENT);
      else
         CRTP_Append_T_Uint8_Data
           (Packet_Handler, ID);
         declare
            V : Parameter_Variable
            renames Parameters.Variables (Integer (ID)).all;
         begin
            case V.Parameter_Type.Size is
               when One_Byte =>
                  declare
                     procedure Append_Data is new CRTP.Append_Data
                       (Types.T_Uint8);
                     Variable : Types.T_Uint8 with Address => V.Variable;
                  begin
                     Append_Data (Packet_Handler, Variable);
                  end;
               when Two_Bytes =>
                  declare
                     procedure Append_Data is new CRTP.Append_Data
                       (Types.T_Uint16);
                     Variable : Types.T_Uint16 with Address => V.Variable;
                  begin
                     Append_Data (Packet_Handler, Variable);
                  end;
               when Four_Bytes =>
                  declare
                     procedure Append_Data is new CRTP.Append_Data
                       (Types.T_Uint32);
                     Variable : Types.T_Uint32 with Address => V.Variable;
                  begin
                     Append_Data (Packet_Handler, Variable);
                  end;
               when Eight_Bytes =>
                  declare
                     procedure Append_Data is new CRTP.Append_Data
                       (Types.T_Uint64);
                     Variable : Types.T_Uint64 with Address => V.Variable;
                  begin
                     Append_Data (Packet_Handler, Variable);
                  end;
            end case;
         end;
      end if;

      CRTP.Send_Packet
        (CRTP.Get_Packet_From_Handler (Packet_Handler),
         One_Off => True);
   end Read_Process;

   -------------------
   -- Write_Process --
   -------------------

   procedure Write_Process (Packet : CRTP.Packet) is
      ID : constant Types.T_Uint8 := Packet.Data_1 (1);
   begin
      if Natural (ID) >= Parameters.Variables_Count then
         --  Invalid
         declare
            Packet_Handler : CRTP.Packet_Handler :=
              CRTP.Create_Packet (CRTP.PORT_PARAM,
                                  Channel'Enum_Rep (WRITE_CH));
            procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
              (Types.T_Uint8);
         begin
            CRTP_Append_T_Uint8_Data (Packet_Handler, Types.T_Uint8'Last);
            CRTP_Append_T_Uint8_Data (Packet_Handler, ID);
            CRTP_Append_T_Uint8_Data (Packet_Handler, ENOENT);
            CRTP.Send_Packet
              (CRTP.Get_Packet_From_Handler (Packet_Handler),
               One_Off => True);
         end;
      else
         declare
            Packet_Handler : constant CRTP.Packet_Handler
              := CRTP.Get_Handler_From_Packet (Packet);
            V : Parameter_Variable
              renames Parameters.Variables (Integer (ID)).all;
         begin
            if V.Parameter_Type.Read_Only then
               raise Program_Error with "attempt to write r/o param";
            end if;
            case V.Parameter_Type.Size is
               when One_Byte =>
                  declare
                     procedure Get_Data is new CRTP.Get_Data
                       (Types.T_Uint8);
                     Variable : Types.T_Uint8 with Address => V.Variable;
                  begin
                     Get_Data (Packet_Handler, 1, Variable);
                  end;
               when Two_Bytes =>
                  declare
                     procedure Get_Data is new CRTP.Get_Data
                       (Types.T_Uint16);
                     Variable : Types.T_Uint16 with Address => V.Variable;
                  begin
                     Get_Data (Packet_Handler, 1, Variable);
                  end;
               when Four_Bytes =>
                  declare
                     procedure Get_Data is new CRTP.Get_Data
                       (Types.T_Uint32);
                     Variable : Types.T_Uint32 with Address => V.Variable;
                  begin
                     Get_Data (Packet_Handler, 1, Variable);
                  end;
               when Eight_Bytes =>
                  declare
                     procedure Get_Data is new CRTP.Get_Data
                       (Types.T_Uint64);
                     Variable : Types.T_Uint64 with Address => V.Variable;
                  begin
                     Get_Data (Packet_Handler, 1, Variable);
                  end;
            end case;
         end;
      end if;

      CRTP.Send_Packet (Packet, One_Off => True);
   end Write_Process;

end Parameter;
