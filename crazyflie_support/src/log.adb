------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2018, AdaCore                     --
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
with Ada.Containers.Bounded_Hashed_Maps;
with Ada.Real_Time;
with Ada.Real_Time.Timing_Events;
with Ada.Strings.Bounded;

with Config;
with CRTP;
with Types;

with CRC;
with Generic_Vectors;

package body Log is

   --  Declarations previously in private part of spec

   --  Type representing all the available log module CRTP channels.
   type Channel is
     (TOC_CH,
      CONTROL_CH,
      DATA_CH);
   for Channel use
     (TOC_CH      => 0,
      CONTROL_CH  => 1,
      DATA_CH     => 2);
   for Channel'Size use 2;

   --  Type representing all the log commands.
   --  CMD_GET_INFO is requested at connexion to fetch the TOC.
   --  CMD_GET_ITEM is requested at connexion to fetch data (name,
   --  type) of a variable.
   type TOC_Command is
     (CMD_GET_ITEM,
      CMD_GET_INFO);
   for TOC_Command use
     (CMD_GET_ITEM => 0,
      CMD_GET_INFO => 1);
   for TOC_Command'Size use 8;

   --  Type representing all the available log control commands.
   type Control_Command is
     (CONTROL_CREATE_BLOCK,
      CONTROL_APPEND_BLOCK,
      CONTROL_DELETE_BLOCK,
      CONTROL_START_BLOCK,
      CONTROL_STOP_BLOCK,
      CONTROL_RESET);
   for Control_Command use
     (CONTROL_CREATE_BLOCK => 0,
      CONTROL_APPEND_BLOCK => 1,
      CONTROL_DELETE_BLOCK => 2,
      CONTROL_START_BLOCK  => 3,
      CONTROL_STOP_BLOCK   => 4,
      CONTROL_RESET        => 5);
   for Control_Command'Size use 8;

   --  Global variables and constants

   --  Constant array registering the length of each log variable type
   --  in Bytes.
   Type_Length : constant array (Variable_Type) of Types.T_Uint8
     := (UINT8  => 1,
         UINT16 => 2,
         UINT32 => 4,
         INT8   => 1,
         INT16  => 2,
         INT32  => 4,
         FLOAT  => 4);

   --  Error code constants
   ENOENT : constant := 2;
   E2BIG  : constant := 7;
   ENOMEM : constant := 12;
   EEXIST : constant := 17;

   --  Maximum number of groups we can create.
   MAX_NUMBER_OF_GROUPS              : constant := 20;
   --  Maximum number of variables we can create inside a group.
   MAX_NUMBER_OF_VARIABLES_PER_GROUP : constant := 8;
   --  Maximum number of variables we can log at the same time.
   MAX_OPS                           : constant := 128;
   --  Maximum number of blocks we can create.
   MAX_BLOCKS                        : constant := 16;

   --  Types

   package Strings
   is new Ada.Strings.Bounded.Generic_Bounded_Length (Max => Max_Name_Length);
   function "+" (B : Strings.Bounded_String) return String
                 renames Strings.To_String;

   subtype Name is Strings.Bounded_String;
   function String_To_Name (S : String) return Name is
     (Strings.To_Bounded_String (S))
     with Pre => S'Length <= Max_Name_Length;

   subtype Group_Identifier is Natural range 0 .. MAX_NUMBER_OF_GROUPS - 1;

   --  Type representing a log variable.
   type Variable is record
      Group_ID : Group_Identifier;
      Nam      : Name;
      Typ      : Variable_Type;
      Variable : System.Address := System.Null_Address;
   end record;

   --  Storage for all the variables.

   Max_Variables : constant
     := MAX_NUMBER_OF_VARIABLES_PER_GROUP * MAX_NUMBER_OF_GROUPS;

   subtype Variable_Identifier is Natural range 0 .. Max_Variables - 1;

   package Variables is new Generic_Vectors
     (Element_Type => Variable);

   subtype Variable_Array is Variables.Vector
     (Capacity => Max_Variables);

   --  Storage for groups. A group has a name and contains a number of
   --  variables; a variable has a name, a type, and an address. A
   --  variable can only appear in one group, though there can be more
   --  than one variable referencing the same address. Groups are
   --  recognised by the client, and for a group to be used it must
   --  contain all the variables expected.

   package Group_Variables is new Generic_Vectors
     (Element_Type => Variable_Identifier);

   subtype Group_Variable_Array is Group_Variables.Vector
     (Capacity => MAX_NUMBER_OF_VARIABLES_PER_GROUP);

   --  Type representing a log group.
   type Group is record
      Nam       : Name;
      Variables : Group_Variable_Array;
   end record;

   package Groups is new Generic_Vectors
     (Element_Type => Group);

   subtype Group_Array is Groups.Vector
     (Capacity => MAX_NUMBER_OF_GROUPS);

   --  Type representing the log TOC

   type TOC is record
      Groups    : Group_Array;
      Variables : Variable_Array;
   end record;

   --  Storage for Blocks.

   --  A Block is requested by the client, using a client-determined
   --  ID (type Types.T_Uint8). It is a request to report a number of
   --  variables at an interval.
   --
   --  A particular variable can appear in more than one block.

   type Operation is record     -- XXX might need to support addresses too
      Variable  : Variable_Identifier;
      Stored_As : Variable_Type;
      Report_As : Variable_Type;
   end record;

   --  subtype Operation_Identifier is Natural range 0 .. MAX_OPS - 1;
   package Operations is new Generic_Vectors
     (Element_Type => Operation);
   subtype Operations_Array is Operations.Vector
     (Capacity => MAX_OPS);

   --  Extension of the Timing_Event tagged type to store additional
   --  attributes : the client's block ID, and how often.
   type Block_Timing_Event is new Ada.Real_Time.Timing_Events.Timing_Event
     with record
      Client_Block_ID : Types.T_Uint8;
      Period          : Ada.Real_Time.Time_Span;
   end record;

   --  Type representing a log block. A log block sends all
   --  its variables' data every time the Timing_Event timer expires.
   type Block is record
      Free       : Boolean := True;
      Timer      : Block_Timing_Event;
      Operations : Operations_Array;
   end record;

   --  We use a map from the client's block ID to ours.
   function Uint8_Hash (Key : Types.T_Uint8) return Ada.Containers.Hash_Type
   is (Ada.Containers.Hash_Type (Key));

   subtype Block_ID is Integer range 0 .. MAX_BLOCKS - 1;

   package Block_Maps is new Ada.Containers.Bounded_Hashed_Maps
     (Key_Type        => Types.T_Uint8,
      Element_Type    => Block_ID,
      Hash            => Uint8_Hash,
      Equivalent_Keys => Types."=");

   --  Type used to encode timestamps when sending log data.
   subtype Time_Stamp is Types.T_Uint8_Array (1 .. 3);

   --  Tasks and protected objects

   task Log_Task
     with Priority => Config.LOG_TASK_PRIORITY;

   protected Block_Timing_Event_Handler is
      pragma Interrupt_Priority;

      procedure Run_Block
        (Event : in out Ada.Real_Time.Timing_Events.Timing_Event);
      entry Get_Block_To_Run (ID : out Types.T_Uint8);

   private
      Record_Required : Boolean := False;
      Block_To_Record : Types.T_Uint8;  -- OK to lose records on overrun
   end Block_Timing_Event_Handler;

   --  Global variables and constants

   Block_Timer_Handler :
   constant Ada.Real_Time.Timing_Events.Timing_Event_Handler
     := Block_Timing_Event_Handler.Run_Block'Access;

   Is_Init : Boolean := False;

   --  Log TOC
   Data : aliased TOC;

   --  Log blocks
   Blocks : array (Block_ID) of Block;

   --  Map from client block ID to ours
   Block_Map : Block_Maps.Map (Capacity => MAX_BLOCKS,
                               Modulus  => MAX_BLOCKS);

   --  Procedures and functions

   --  Handler called when a CRTP packet is received in the log
   --  port.
   procedure CRTP_Handler (Packet : CRTP.Packet);

   --  Process a command related to TOC demands from the python client.
   procedure TOC_Process (Packet : CRTP.Packet);

   --  Process a command related to log control.
   procedure Control_Process (Packet : CRTP.Packet);

   --  Create a log block, contatining all the variables specified
   --  in Ops_Settings parameter.
   function Create_Block
     (ID               : Types.T_Uint8;
      Ops_Settings_Raw : Types.T_Uint8_Array) return Types.T_Uint8;

   --  Delete the specified block.
   function Delete_Block (ID : Types.T_Uint8) return Types.T_Uint8;
   procedure Delete_Block (ID : Block_ID);

   --  Append the variables specified in Ops_Settings to the
   --  block identified with Block_ID.
   function Append_To_Block
     (ID               : Types.T_Uint8;
      Ops_Settings_Raw : Types.T_Uint8_Array) return Types.T_Uint8;

   --  Start logging the specified block at each period (in ms).
   function Start_Block
     (ID     : Types.T_Uint8;
      Period : Natural) return Types.T_Uint8;

   --  Stop logging the specified block.
   function Stop_Block (ID : Types.T_Uint8) return Types.T_Uint8;

   --  Delete all the log blocks.
   procedure Reset;

   --  Calculate the current block length, to ensure that it will fit in
   --  a single CRTP packet.
   function Calculate_Block_Length (Blk : Block) return Types.T_Uint8;
   pragma Inline (Calculate_Block_Length);

   --  Get a log timestamp from the current clock tick count.
   function Get_Time_Stamp return Time_Stamp;
   pragma Inline (Get_Time_Stamp);

   --  Append raw data from the variable and group name.
   procedure Append_Raw_Data_Variable_Name_To_Packet
     (Group_Name     : String;
      Variable_Name  : String;
      Packet_Handler : in out CRTP.Packet_Handler);

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      CRTP.Register_Callback (CRTP.PORT_LOG, CRTP_Handler'Access);

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   ----------------------
   -- Create_Group --
   ----------------------

   procedure Create_Group
     (Name     :     String;
      Group_ID : out Natural;
      Success  : out Boolean)
   is
   begin
      Success := False;

      if Data.Groups.Length = MAX_NUMBER_OF_GROUPS then
         return;
      end if;

      Data.Groups.Append
        ((Nam   => String_To_Name (Name),
          others => <>));

      Group_ID := Data.Groups.Length - 1;

      Success := True;
   end Create_Group;

   ----------------------------------
   -- Append_Variable_To_Group --
   ----------------------------------

   procedure Append_Variable_To_Group
     (Group_ID :     Natural;
      Name     :     String;
      Typ      :     Variable_Type;
      Variable :     System.Address;
      Success  : out Boolean)
   is
      Grp : Group
      renames Data.Groups.Element_Access (Group_ID).all;
   begin
      Success := False;

      if Grp.Variables.Length = MAX_NUMBER_OF_VARIABLES_PER_GROUP
      then
         return;
      end if;

      Data.Variables.Append ((Group_ID => Group_ID,
                              Nam      => String_To_Name (Name),
                              Typ      => Typ,
                              Variable => Variable));

      Grp.Variables.Append (Data.Variables.Length - 1);

      Success := True;
   end Append_Variable_To_Group;

   ----------------------
   -- Add_Variable --
   ----------------------

   procedure Add_Variable
     (Group    :     String;
      Name     :     String;
      Typ      :     Variable_Type;
      Variable :     System.Address;
      Success  : out Boolean)
   is
      Group_Name : constant Log.Name := String_To_Name (Group);
      Group_ID : Integer := -1;
      use type Log.Name;
   begin
      Success := True;
      --  May be falsified if a new group is required but can't be
      --  created.

      for J in 0 .. Data.Groups.Length - 1 loop
         if Data.Groups.Element (J).Nam = Group_Name then
            Group_ID := J;
            exit;
         end if;
      end loop;

      if Group_ID not in Group_Identifier then
         --  This will be a new group; create it.
         Create_Group (Name     => Group,
                       Group_ID => Group_ID,
                       Success  => Success);
      end if;

      --  Add the variable (if all OK so far).
      if Success then
         Append_Variable_To_Group (Group_ID => Group_ID,
                                   Name     => Name,
                                   Typ      => Typ,
                                   Variable => Variable,
                                   Success  => Success);
      end if;
   end Add_Variable;

   --  Private procedures and functions

   ----------------------
   -- CRTP_Handler --
   ----------------------

   procedure CRTP_Handler (Packet : CRTP.Packet)
   is
      Ch : constant Channel := Channel'Val (Packet.Channel);
   begin
      case Ch is
         when TOC_CH =>
            TOC_Process (Packet);
         when CONTROL_CH =>
            Control_Process (Packet);
         when others =>
            null;
      end case;
   end CRTP_Handler;

   ---------------------
   -- TOC_Process --
   ---------------------

   procedure TOC_Process (Packet : CRTP.Packet)
   is
      procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
        (Types.T_Uint8);

      procedure CRTP_Append_T_Uint32_Data is new CRTP.Append_Data
        (Types.T_Uint32);

      function Data_CRC32 return Types.T_Uint32;
      function Data_CRC32 return Types.T_Uint32 is
         --  Note, this doesn't take account of uninitialized
         --  components of Data, but since the only use (in
         --  cfclient, anyway) is to determine whether to load new
         --  data this will just add a minor startup load.
         function TOC_CRC is new CRC.Make (Data_Kind => TOC);
      begin
         return Types.T_Uint32 (TOC_CRC (Data));
      end Data_CRC32;

      Command        : constant TOC_Command
        := TOC_Command'Val (Packet.Data_1 (1));
      Packet_Handler : CRTP.Packet_Handler;
   begin
      Packet_Handler := CRTP.Create_Packet
        (CRTP.PORT_LOG, Channel'Enum_Rep (TOC_CH));
      CRTP_Append_T_Uint8_Data
        (Packet_Handler,
         TOC_Command'Enum_Rep (Command));

      case Command is
         when CMD_GET_INFO =>
            CRTP_Append_T_Uint8_Data
              (Packet_Handler,
               Types.T_Uint8 (Data.Variables.Length));

            CRTP_Append_T_Uint32_Data
              (Packet_Handler, Data_CRC32);
            CRTP_Append_T_Uint8_Data
              (Packet_Handler, MAX_BLOCKS);
            CRTP_Append_T_Uint8_Data
              (Packet_Handler, MAX_OPS);

         when CMD_GET_ITEM =>
            declare
               Var_ID : constant Integer
                 := Integer (Packet.Data_1 (2));
            begin
               if Var_ID < Data.Variables.Length then
                  CRTP_Append_T_Uint8_Data
                    (Packet_Handler,
                     Types.T_Uint8 (Var_ID));

                  declare
                     Var : Variable renames
                       Data.Variables.Element_Access (Var_ID).all;
                     Grp : Group renames
                       Data.Groups.Element_Access
                         (Var.Group_ID).all;
                  begin
                     CRTP_Append_T_Uint8_Data
                       (Packet_Handler,
                        Variable_Type'Enum_Rep (Var.Typ));
                     Append_Raw_Data_Variable_Name_To_Packet
                       (+Grp.Nam,
                        +Var.Nam,
                        Packet_Handler);
                  end;
               else
                  --  Return the packet with no content.
                  null;
               end if;
            end;
      end case;
      CRTP.Send_Packet
        (CRTP.Get_Packet_From_Handler (Packet_Handler),
         One_Off => True);
   end TOC_Process;

   -------------------------
   -- Control_Process --
   -------------------------

   procedure Control_Process (Packet : CRTP.Packet)
   is
      ------------------------------------
      -- T_Uint8_To_Control_Command --
      ------------------------------------

      function T_Uint8_To_Control_Command is new Ada.Unchecked_Conversion
        (Types.T_Uint8, Control_Command);

      Tx_Packet   : CRTP.Packet := Packet;
      Command     : Control_Command;
      Answer      : Types.T_Uint8;

      use type Types.T_Uint8;
   begin
      Command := T_Uint8_To_Control_Command (Packet.Data_1 (1));

      case Command is
         when CONTROL_CREATE_BLOCK =>
            Answer := Create_Block
              (ID               => Packet.Data_1 (2),
               Ops_Settings_Raw => Packet.Data_1 (3 .. Integer (Packet.Size)));
         when CONTROL_APPEND_BLOCK =>
            Answer := Append_To_Block
              (ID               => Packet.Data_1 (2),
               Ops_Settings_Raw => Packet.Data_1 (3 .. Integer (Packet.Size)));
         when CONTROL_DELETE_BLOCK =>
            Answer := Delete_Block (Packet.Data_1 (2));
         when CONTROL_START_BLOCK =>
            Answer := Start_Block
              (ID     => Packet.Data_1 (2),
               Period => Integer (Packet.Data_1 (3) *  10));
         when CONTROL_STOP_BLOCK =>
            Answer := Stop_Block (Packet.Data_1 (2));
         when CONTROL_RESET =>
            Reset;
            Answer := 0;
      end case;

      Tx_Packet.Data_1 (3) := Answer;
      Tx_Packet.Size := 3;
      CRTP.Send_Packet (Tx_Packet, One_Off => True);
   end Control_Process;

   ----------------------
   -- Create_Block --
   ----------------------

   function Create_Block
     (ID               : Types.T_Uint8;
      Ops_Settings_Raw : Types.T_Uint8_Array) return Types.T_Uint8
   is
      use type Ada.Containers.Count_Type;
   begin
      --  Not enough memory to create a new block.
      if Block_Map.Length = Block_Map.Capacity then
         return ENOMEM;
      end if;

      --  Block with the same ID already exists.
      if Block_Map.Contains (ID) then
         return EEXIST;
      end if;

      pragma Assert ((for some Block of Blocks => Block.Free),
                     "log create, no free log blocks");

      pragma Warnings
        (Off, """return"" statement missing following this statement");

      --  Find a free block
      for J in Blocks'Range loop
         if Blocks (J).Free then
         --  Map and set up the block.
            Block_Map.Insert (Key => ID, New_Item => J);

            Blocks (J).Free := False;

            return Append_To_Block (ID, Ops_Settings_Raw);
         end if;
      end loop;

      pragma Warnings
        (On, """return"" statement missing following this statement");
   end Create_Block;

   ----------------------
   -- Delete_Block --
   ----------------------

   function Delete_Block (ID : Types.T_Uint8) return Types.T_Uint8
   is
      Cursor : Block_Maps.Cursor := Block_Map.Find (ID);
   begin
      --  ID doesn't match anything
      if not Block_Maps.Has_Element (Cursor) then
         return ENOENT;
      end if;

      Delete_Block (Block_Maps.Element (Cursor));
      Block_Map.Delete (Cursor);

      return 0;
   end Delete_Block;

   procedure Delete_Block (ID : Block_ID)
   is
      Blk   : Block renames Blocks (ID);
      Dummy : Boolean;
   begin
      --  Stop the timer.
      Cancel_Handler (Blk.Timer, Dummy);

      --  Mark the block as a free one.
      Blk.Free := True;
      Blk.Operations.Clear;
   end Delete_Block;

   -------------------------
   -- Append_To_Block --
   -------------------------

   function Append_To_Block
     (ID               : Types.T_Uint8;
      Ops_Settings_Raw : Types.T_Uint8_Array) return Types.T_Uint8
   is
      type Short_Variable_Type is new Variable_Type with Size => 4;

      type Ops_Setting is record
         Storage_Type : Short_Variable_Type;
         Typ          : Short_Variable_Type;
         Variable_ID  : Types.T_Uint8;
      end record
        with Size => 16;
      for Ops_Setting use record
         Storage_Type at 0 range 0 .. 3;
         Typ          at 0 range 4 .. 7;
         Variable_ID  at 1 range 0 .. 7;
      end record;

      type Ops_Settings_Array is
        array (1 .. Ops_Settings_Raw'Length / 2) of Ops_Setting;

      -----------------------------------------
      -- T_Uint8_Array_To_Ops_Settings_Array --
      -----------------------------------------

      function T_Uint8_Array_To_Ops_Settings_Array is
        new Ada.Unchecked_Conversion (Types.T_Uint8_Array, Ops_Settings_Array);

      Cursor : constant Block_Maps.Cursor := Block_Map.Find (ID);
      Blk_ID : Block_ID;
   begin
      --  Block ID doesn't match anything
      if not Block_Maps.Has_Element (Cursor) then
         return ENOENT;
      end if;

      Blk_ID := Block_Maps.Element (Cursor);
      pragma Assert (not Blocks (Blk_ID).Free,
                     "log append, mapped block not free");

      declare
         Blk : Block renames Blocks (Blk_ID);
         Current_Block_Length : Types.T_Uint8;
         Variable             : Variable_Identifier;

         Ops_Settings : constant Ops_Settings_Array
           := T_Uint8_Array_To_Ops_Settings_Array (Ops_Settings_Raw);

         use type Types.T_Uint8;
      begin

         for O of Ops_Settings loop
            Current_Block_Length := Calculate_Block_Length (Blk);

            --  Trying to append a full block
            if Current_Block_Length
              + Type_Length (Variable_Type (O.Typ)) >
              CRTP.MAX_DATA_SIZE
            then
               return E2BIG;
            end if;

            --  Check not trying to add a variable that does not exist
            if Natural (O.Variable_ID) >  Data.Variables.Length
            then
               return ENOENT;
            end if;

            Variable := Variable_Identifier (O.Variable_ID);

            --  XXX Shouldn't we check that O.Storage_Type matches the
            --  variable's Storage_Type?

            Blk.Operations.Append
              ((Variable => Variable,
                Stored_As =>
                  Data.Variables.Element (Variable).Typ,
                Report_As => Variable_Type (O.Typ)));
         end loop;
      end;

      return 0;
   end Append_To_Block;

   ---------------------
   -- Start_Block --
   ---------------------

   function Start_Block
     (ID     : Types.T_Uint8;
      Period : Natural) return Types.T_Uint8
   is
      Cursor : constant Block_Maps.Cursor := Block_Map.Find (ID);
   begin
      --  Block ID doesn't match anything
      if not Block_Maps.Has_Element (Cursor) then
         return ENOENT;
      end if;

      declare
         Blk_ID : constant Block_ID := Block_Maps.Element (Cursor);
         Blk : Block renames Blocks (Blk_ID);
         Dummy : Boolean;

         use type Ada.Real_Time.Time;
      begin
         pragma Assert (not Blk.Free, "log start, mapped block not free");

         if Period > 0 then
            Cancel_Handler (Blk.Timer, Dummy);
            Blk.Timer.Client_Block_ID := ID;
            Blk.Timer.Period :=
              Ada.Real_Time.Milliseconds (Natural'Max (Period, 10));
            Set_Handler (Event   => Blk.Timer,
                         At_Time => Ada.Real_Time.Clock + Blk.Timer.Period,
                         Handler => Block_Timer_Handler);
         else
            --  TODO: single shot run. Use worker task for it.
            null;
         end if;
      end;

      return 0;
   end Start_Block;

   --------------------
   -- Stop_Block --
   --------------------

   function Stop_Block (ID : Types.T_Uint8) return Types.T_Uint8
   is
      Dummy : Boolean;
      Cursor : constant Block_Maps.Cursor := Block_Map.Find (ID);
   begin
      --  Block ID doesn't match anything
      if not Block_Maps.Has_Element (Cursor) then
         return ENOENT;
      end if;

      --  Stop the timer.
      Cancel_Handler (Blocks (Block_Maps.Element (Cursor)).Timer,
                      Dummy);

      return 0;
   end Stop_Block;

   ---------------
   -- Reset --
   ---------------

   procedure Reset is
      Dummy  : Types.T_Uint8;
   begin
      if Is_Init then
         for Block of Block_Map loop
            Delete_Block (Block);
         end loop;
         Block_Map.Clear;
      end if;
   end Reset;

   ---------------------------------------------
   -- Append_Raw_Data_Variable_Name_To_Packet --
   ---------------------------------------------

   procedure Append_Raw_Data_Variable_Name_To_Packet
     (Group_Name     : String;
      Variable_Name  : String;
      Packet_Handler : in out CRTP.Packet_Handler)
   is
      subtype Complete_Name is
        String (1 .. Group_Name'Length + 1 + Variable_Name'Length + 1);
      --  includes 2 nulls
      subtype Complete_Name_Raw is
        Types.T_Uint8_Array (Complete_Name'Range);

      function Complete_Name_To_Complete_Name_Raw is new
        Ada.Unchecked_Conversion (Complete_Name, Complete_Name_Raw);

      procedure CRTP_Append_Complete_Name_Raw_Data is new
        CRTP.Append_Data (Complete_Name_Raw);

      Raw_Complete_Name : constant Complete_Name_Raw
        := Complete_Name_To_Complete_Name_Raw
          (Group_Name & ASCII.NUL & Variable_Name & ASCII.NUL);
   begin
      CRTP_Append_Complete_Name_Raw_Data
        (Packet_Handler, Raw_Complete_Name);
   end Append_Raw_Data_Variable_Name_To_Packet;

   ----------------------------
   -- Calculate_Block_Length --
   ----------------------------

   function Calculate_Block_Length (Blk : Block) return Types.T_Uint8
   is
      Block_Length : Types.T_Uint8 := 0;

      use type Types.T_Uint8;
   begin

      for J in 0 .. Blk.Operations.Length - 1 loop
         Block_Length := Block_Length
           + Type_Length (Blk.Operations.Element (J).Report_As);
      end loop;

      return Block_Length;
   end Calculate_Block_Length;

   ------------------------
   -- Get_Time_Stamp --
   ------------------------

   function Get_Time_Stamp return Time_Stamp
   is
      subtype  Time_T_Uint8_Array is Types.T_Uint8_Array (1 .. 8);

      --------------------------------------
      -- Time_To_T_Uint8_Array --
      --------------------------------------

      function Time_To_Time_T_Uint8_Array is new Ada.Unchecked_Conversion
        (Ada.Real_Time.Time, Time_T_Uint8_Array);

      Raw_Time : Time_T_Uint8_Array;
   begin
      Raw_Time := Time_To_Time_T_Uint8_Array (Ada.Real_Time.Clock);

      return Raw_Time (6 .. 8);
   end Get_Time_Stamp;

   --  Tasks and protected objects

   ------------------------------------
   -- Block_Timing_Event_Handler --
   ------------------------------------

   protected body Block_Timing_Event_Handler is

      -------------------
      -- Run_Block --
      -------------------

      procedure Run_Block
        (Event : in out Ada.Real_Time.Timing_Events.Timing_Event) is
         Actual_Event : Block_Timing_Event
         renames Block_Timing_Event
           (Ada.Real_Time.Timing_Events.Timing_Event'Class (Event));

         use type Ada.Real_Time.Time;
      begin
         Block_To_Record := Actual_Event.Client_Block_ID;
         Record_Required := True;

         Ada.Real_Time.Timing_Events.Set_Handler
           (Event   => Event,
            At_Time => Ada.Real_Time.Clock + Actual_Event.Period,
            Handler => Block_Timer_Handler);
      end Run_Block;

      ---------------
      -- Run_Block --
      ---------------

      entry Get_Block_To_Run (ID : out Types.T_Uint8) when Record_Required is
      begin
         Record_Required := False;
         ID := Block_To_Record;
      end Get_Block_To_Run;

   end Block_Timing_Event_Handler;

   task body Log_Task is
      Client_Block_ID : Types.T_Uint8;
      Blk_ID          : Block_ID;
      Variable        : System.Address;
      Time_Stmp       : Time_Stamp;
      Packet_Handler  : CRTP.Packet_Handler;
      Success         : Boolean
        with Unreferenced;

      --  Procedures used to append log data with different types

      ------------------------------------
      -- Procedures used to append data --
      ------------------------------------

      procedure CRTP_Append_Time_Stamp_Data is new CRTP.Append_Data
        (Time_Stamp);
      procedure CRTP_Append_T_Uint8_Data is new CRTP.Append_Data
        (Types.T_Uint8);
      procedure CRTP_Append_T_Uint16_Data is new CRTP.Append_Data
        (Types.T_Uint16);
      procedure CRTP_Append_T_Uint32_Data is new CRTP.Append_Data
        (Types.T_Uint32);
      procedure CRTP_Append_T_Int8_Data is new CRTP.Append_Data
        (Types.T_Int8);
      procedure CRTP_Append_T_Int16_Data is new CRTP.Append_Data
        (Types.T_Int16);
      procedure CRTP_Append_T_Int32_Data is new CRTP.Append_Data
        (Types.T_Int32);
      procedure CRTP_Append_Float_Data is new CRTP.Append_Data
        (Standard.Float);
   begin
      loop
         Block_Timing_Event_Handler.Get_Block_To_Run
           (ID => Client_Block_ID);

         if CRTP.Is_Connected
           and then Block_Map.Contains (Client_Block_ID)
           --  Bit of a race condition here! Blocks could have been reset.
         then

            Blk_ID := Block_Map (Client_Block_ID);

            Time_Stmp := Get_Time_Stamp;

            Packet_Handler :=
              CRTP.Create_Packet
                (Port    => CRTP.PORT_LOG,
                 Channel => Channel'Enum_Rep (DATA_CH));

            --  Add block ID to the packet.
            CRTP_Append_T_Uint8_Data (Handler     => Packet_Handler,
                                      Data        => Client_Block_ID);
            --  Add a timestamp to the packet
            CRTP_Append_Time_Stamp_Data (Handler     => Packet_Handler,
                                         Data        => Time_Stmp);

            --  Add all the variables data in the packet
            for J in 0 .. Blocks (Blk_ID).Operations.Length - 1 loop
               declare
                  Op : Operation renames
                    Blocks (Blk_ID).Operations.Element_Access (J).all;
               begin
                  --  Get the address of this operation's variable.
                  Variable :=
                    Data.Variables.Element (Op.Variable).Variable;

                  case Op.Report_As is
                     when UINT8 =>
                        declare
                           Value : Types.T_Uint8;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Uint8_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                     when UINT16 =>
                        declare
                           Value : Types.T_Uint16;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Uint16_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                     when UINT32 =>
                        declare
                           Value : Types.T_Uint32;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Uint32_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                     when INT8 =>
                        declare
                           Value : Types.T_Int8;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Int8_Data
                             (Handler      => Packet_Handler,
                              Data        => Value);
                        end;
                     when INT16 =>
                        declare
                           Value : Types.T_Int16;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Int16_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                     when INT32 =>
                        declare
                           Value : Types.T_Int32;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_T_Int32_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                     when FLOAT =>
                        declare
                           Value : Standard.Float;
                           for Value'Address use Variable;
                        begin
                           CRTP_Append_Float_Data
                             (Handler     => Packet_Handler,
                              Data        => Value);
                        end;
                  end case;
               end;
            end loop;

            CRTP.Send_Packet
              (Pkt     => CRTP.Get_Packet_From_Handler (Packet_Handler),
               One_Off => False);

         else -- CRTP no longer connected
            Reset;
            CRTP.Reset;
         end if;

      end loop;
   end Log_Task;

end Log;
