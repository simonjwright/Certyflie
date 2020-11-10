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

with Ada.Numerics.Elementary_Functions;
with Ada.Real_Time;
with Ada.Synchronous_Task_Control;
with HAL.GPIO;
with HAL.SPI;
with Ravenscar_Time;
with Interfaces;
with STM32.Board;
with STM32.Device;
with STM32.GPIO;
with STM32.I2C;
with STM32.SPI;

with PMW3901;
with VL53L0X;

with Estimators;
--  with Parameter;
--  with Log;

with Console;
with Semihosting;

package body Flow_Deck is

   use type STM32.GPIO.GPIO_Point;

   Flow_Sensor_Port : STM32.SPI.SPI_Port renames STM32.Board.EXT_SPI;
   Flow_Select_Port : STM32.GPIO.GPIO_Point renames STM32.Board.EXT_CS1;
   Flow_Sensor : PMW3901.PMW3901_Flow_Sensor
     (Port   => Flow_Sensor_Port'Access,
      CS     => Flow_Select_Port'Access,
      Timing => Ravenscar_Time.Delays);

   ToF_Sensor_Port : STM32.I2C.I2C_Port renames STM32.Device.I2C_1;
   ToF_Sensor      : VL53L0X.VL53L0X_Ranging_Sensor
     (Port   => ToF_Sensor_Port'Access,
      Timing => Ravenscar_Time.Delays);

   Flow_Sensor_Initialized : Boolean := False with Convention => C;
   ToF_Sensor_Initialized  : Boolean := False with Convention => C;

   procedure Initialize_EXT_SPI;
   procedure Configure_EXT_CS (Pin : in out STM32.GPIO.GPIO_Point)
   with Pre =>
     Pin = STM32.Board.EXT_CS0 or
     Pin = STM32.Board.EXT_CS1 or
     Pin = STM32.Board.EXT_CS2 or
     Pin = STM32.Board.EXT_CS3;

   --  procedure Init_Logging;
   --  procedure Init_Parameters;

   procedure Init_Flow_Sensor (Success : out Boolean);
   procedure Init_ToF_Sensor (Success : out Boolean);

   --  For debug
   procedure Put_Line (S : String);
   procedure Put_Line (S : String) is
   begin
      Console.Put_Line (S);
      Semihosting.Log_Line (S);
   end Put_Line;

   Flow_Sensor_SO : Ada.Synchronous_Task_Control.Suspension_Object;
   ToF_Sensor_SO  : Ada.Synchronous_Task_Control.Suspension_Object;

   procedure Init is
   begin
      --  Init_Logging;
      --  Init_Parameters;
      Init_Flow_Sensor (Flow_Sensor_Initialized);
      Init_ToF_Sensor (ToF_Sensor_Initialized);

      --  Only start processing if both sensors started OK
      if Flow_Sensor_Initialized and ToF_Sensor_Initialized then
         Put_Line ("Flow & ToF both initialized");
         Ada.Synchronous_Task_Control.Set_True (Flow_Sensor_SO);
         Ada.Synchronous_Task_Control.Set_True (ToF_Sensor_SO);
      end if;
   end Init;

   function Test return Boolean is
     (Flow_Sensor_Initialized and ToF_Sensor_Initialized);

   -----------------------
   -- Local subprograms --
   -----------------------
   procedure Initialize_EXT_SPI
   is
      use STM32.Board;
      EXT_SPI_Points : constant STM32.GPIO.GPIO_Points := (EXT_MISO,
                                                           EXT_MOSI,
                                                           EXT_SCK);

   begin
      STM32.Device.Enable_Clock (EXT_SPI_Points);
      STM32.GPIO.Configure_IO -- values copied from openmv.adb
        (EXT_SPI_Points,
         (Mode           => STM32.GPIO.Mode_AF,
          AF             => STM32.Device.GPIO_AF_SPI1_5,
          Resistors      => STM32.GPIO.Pull_Down,  --  SPI low polarity?
          AF_Speed       => STM32.GPIO.Speed_50MHz,
          AF_Output_Type => STM32.GPIO.Push_Pull));

      STM32.Device.Enable_Clock (EXT_SPI);
      EXT_SPI.Disable;
      EXT_SPI.Configure
        ((Direction           => STM32.SPI.D2Lines_FullDuplex,
          Mode                => STM32.SPI.Master,
          Data_Size           => HAL.SPI.Data_Size_8b,
          Clock_Polarity      => STM32.SPI.Low,
          Clock_Phase         => STM32.SPI.P1Edge,
          Slave_Management    => STM32.SPI.Software_Managed,
          Baud_Rate_Prescaler => STM32.SPI.BRP_64,
          First_Bit           => STM32.SPI.MSB,
          CRC_Poly            => 0));
      EXT_SPI.Enable;
   end Initialize_EXT_SPI;

   procedure Configure_EXT_CS (Pin : in out STM32.GPIO.GPIO_Point)
   is
   begin
      STM32.Device.Enable_Clock (Pin);
      STM32.GPIO.Configure_IO
        (Pin,
         (Mode        => STM32.GPIO.Mode_Out,
          Resistors   => STM32.GPIO.Floating,
          Output_Type => STM32.GPIO.Push_Pull,
          Speed       => STM32.GPIO.Speed_50MHz));
   end Configure_EXT_CS;

   procedure Init_Flow_Sensor (Success : out Boolean) is
   begin
      Initialize_EXT_SPI;
      Configure_EXT_CS (STM32.Board.EXT_CS1);

      PMW3901.Initialize (Flow_Sensor);
      Success := PMW3901.Is_Initialized (Flow_Sensor);
      if not Success then
         Put_Line ("pwm3901 failed");
         return;
      end if;

      PMW3901.Calibrate (Flow_Sensor);
   end Init_Flow_Sensor;

   procedure Init_ToF_Sensor (Success : out Boolean) is
      ID : HAL.UInt16;
      Revision : HAL.UInt8;

      use HAL;
   begin
      STM32.Board.Initialize_I2C_GPIO (ToF_Sensor_Port);
      STM32.Board.Configure_I2C (ToF_Sensor_Port);

      VL53L0X.Initialize (ToF_Sensor);

      ID := VL53L0X.Read_Id (ToF_Sensor);
      Revision := VL53L0X.Read_Revision (ToF_Sensor);

      Success := ID = 16#eeaa# and Revision = 16#10#;
      if not Success then
         Put_Line ("vl53l0x not recognised");
         return;
      end if;

      VL53L0X.Data_Init (ToF_Sensor, Success);
      if not Success then
         Put_Line ("vl53l0x data init failed");
         return;
      end if;

      VL53L0X.Static_Init (ToF_Sensor,
                           GPIO_Function => VL53L0X.New_Sample_Ready,
                           Status => Success);
      if not Success then
         Put_Line ("vl53l0x static init failed");
         return;
      end if;

      VL53L0X.Set_VCSEL_Pulse_Period_Pre_Range (ToF_Sensor,
                                                Period => 18,
                                                Status => Success);
      if not Success then
         Put_Line ("vl53l0x set pre range failed");
         return;
      end if;

      VL53L0X.Set_VCSEL_Pulse_Period_Final_Range (ToF_Sensor,
                                                  Period => 14,
                                                  Status => Success);
      if not Success then
         Put_Line ("vl53l0x set final range failed");
         return;
      end if;

      VL53L0X.Perform_Ref_Calibration (ToF_Sensor,
                                       Status => Success);
      if not Success then
         Put_Line ("vl53l0x ref calibration failed");
         return;
      end if;

   end Init_ToF_Sensor;

   ------------------
   -- Logging data --
   ------------------

   --  Last_Motion : PMW3901.Motion;

   ----------------
   -- Parameters --
   ----------------

   --  Motion_Disabled : Boolean := True with Convention => C;
   --  Don't pass data to the Estimator.

   -----------
   -- Tasks --
   -----------
   --  These priorities are those used by Bitcraze.

   task Flow_Sensor_Task with Priority => 3;
   task ToF_Task with Priority => 2;

   task body Flow_Sensor_Task is
      Outlier_Limit : constant := 100;
      M : PMW3901.Motion;
      use type Ada.Real_Time.Time;
      use type Interfaces.Integer_16;
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (Flow_Sensor_SO);
      Put_Line ("Flow_Sensor_Task started");
      loop
         M := PMW3901.Read_Motion (Flow_Sensor);
         --  Last_Motion := M; -- for logging
         if PMW3901.Is_Valid (M) then
            if abs M.Delta_X < Outlier_Limit
              and then abs M.Delta_Y < Outlier_Limit
            then
               --  Provide the current measurement, flipping to
               --  account for the sensor mounting.
               Estimators.Current_Estimator.Enqueue
                 (Estimators.Flow_Measurement'
                    (Timestamp            => Ada.Real_Time.Clock,
                     Dx                   => Float (-M.Delta_Y),
                     Dy                   => Float (-M.Delta_X),
                     Standard_Deviation_X => 0.25,
                     Standard_Deviation_Y => 0.25,
                     Dt                   => 0.01));
            end if;

         end if;
         delay until Ada.Real_Time.Clock + Ada.Real_Time.Milliseconds (10);
      end loop;
   end Flow_Sensor_Task;

   task body ToF_Task is
      --  data from src/deck/drivers/src/zranger.c
      Exp_Point_A : constant       := 1.0;    -- m
      Exp_Std_A   : constant       := 0.0025; -- SD at 1 m
      Exp_Point_B : constant       := 1.3;    -- m
      Exp_Std_B   : constant       := 0.2;    -- SD at 1.3 m
      Exp_Coeff   : constant Float :=
        Ada.Numerics.Elementary_Functions.Log
          (Exp_Std_B / Exp_Std_A) / (Exp_Point_B - Exp_Point_A);

      Outlier_Limit : constant := 3_000;  -- mm

      Status : Boolean;
      use type Ada.Real_Time.Time;
      use type HAL.UInt16;
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (ToF_Sensor_SO);
      Put_Line ("ToF_Sensor_Task started");
      VL53L0X.Start_Continuous
        (ToF_Sensor, Period_Ms => 20, Status => Status);
      if not Status then
         delay until Ada.Real_Time.Time_Last;
      end if;
      loop
         delay until Ada.Real_Time.Clock + Ada.Real_Time.Milliseconds (33);
         while not VL53L0X.Range_Value_Available (ToF_Sensor) loop
            delay until Ada.Real_Time.Clock
              + Ada.Real_Time.Milliseconds (1);
         end loop;
         declare
            Height : constant HAL.UInt16
              := VL53L0X.Read_Range_Millimeters (ToF_Sensor);
         begin
            if Height < Outlier_Limit then
               declare
                  Distance : constant Float := Float (Height) / 1_000.0; -- m
                  SD : constant Float
                    := Exp_Std_A *
                      (1.0 + Ada.Numerics.Elementary_Functions.Exp
                         (Exp_Coeff * (Distance - Exp_Point_A)));
               begin
                  Estimators.Current_Estimator.Enqueue
                    (Estimators.ToF_Measurement'
                       (Timestamp          => Ada.Real_Time.Clock,
                        Distance           => Distance,
                        Standard_Deviation => SD));
               end;
            end if;
         end;
      end loop;
   end ToF_Task;

   --  procedure Init_Logging is
   --     Success : Boolean;
   --  begin
   --     Log.Add_Variable (Group    => "motion",
   --                       Name     => "deltaX",
   --                       Typ      => Log.INT16,
   --                       Variable => Last_Motion.Delta_X'Address,
   --                       Success  => Success);
   --     pragma Assert (Success, "coudn't add log 'motion.deltaX'");
   --     Log.Add_Variable (Group    => "motion",
   --                       Name     => "deltaY",
   --                       Typ      => Log.INT16,
   --                       Variable => Last_Motion.Delta_Y'Address,
   --                       Success  => Success);
   --     pragma Assert (Success, "coudn't add log 'motion.deltaY'");
   --  end Init_Logging;

   --  procedure Init_Parameters is
   --     Motion_Group_ID : Natural;
   --     Deck_Group_ID : Natural;
   --     Success : Boolean;
   --     use Parameter;
   --  begin
   --     Create_Parameter_Group ("motion",
   --                             Group_ID => Motion_Group_ID,
   --                             Has_Succeed => Success);
   --     pragma Assert (Success, "coudn't create 'motion' parameter group");
   --     Append_Parameter_Variable_To_Group
   --       (Motion_Group_ID,
   --        Name => "disable",
   --        Parameter_Type => Parameter_Variable_Type'
   --          (Size => One_Byte,
   --           Floating => False,
   --           Signed => False,
   --           Read_Only => False,
   --           others => <>),
   --        Variable => Motion_Disabled'Address,
   --        Has_Succeed => Success);
   --     pragma Assert (Success,
   --                    "couldn't create 'motion.disable' parameter");

   --     Create_Parameter_Group ("deck",
   --                             Group_ID => Deck_Group_ID,
   --                             Has_Succeed => Success);
   --     pragma Assert (Success, "coudn't create 'deck' parameter group");
   --     Append_Parameter_Variable_To_Group
   --       (Deck_Group_ID,
   --        Name => "bcFlow",
   --        Parameter_Type => Parameter_Variable_Type'
   --          (Size => One_Byte,
   --           Floating => False,
   --           Signed => False,
   --           Read_Only => True,
   --           others => <>),
   --        Variable => ToF_Sensor_Initialized'Address,
   --        Has_Succeed => Success);
   --     pragma Assert (Success, "couldn't create 'deck.bcFlow' parameter");
   --  end Init_Parameters;

end Flow_Deck;
