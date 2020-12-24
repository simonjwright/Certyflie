------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2016, AdaCore                     --
--           Copyright (C) 2020, Simon Wright <simon@pushface.org>          --
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

with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;
with Ada.Synchronous_Task_Control;

with STM32.Board;
with STM32.I2C;
with Crazyflie_Config;
with Console;
with Parameter;

with AK8963;

with LPS25H.I2C;
with Ravenscar_Time;

package body IMU is

   --  Both MPU9250 and LPS25H use I2C3 (known here as
   --  STM32.Board.MPU_Device, not how I would have done it I think?)
   I2C_Device_SO : Ada.Synchronous_Task_Control.Suspension_Object;

   --  Barometer-related stuff

   LPS25H_Sensor : LPS25H.I2C.LPS25H_Barometric_Sensor_I2C
     (Port              => STM32.Board.MPU_Device.Port,
      Slave_Address_Odd => True,
      Timing            => Ravenscar_Time.Delays);

   --  Local procedures

   procedure Read_6_Unprotected
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data);

   procedure Initialize_Parameter_Logging;

   --------------
   -- Init --
   --------------

   procedure Init (Use_Mag    : Boolean;
                   DLPF_256Hz : Boolean) is
      Delay_After_Reset_Time : Ada.Real_Time.Time;
      use type Ada.Real_Time.Time;
   begin
      if Is_Init then
         return;
      end if;

      --  This should be the first call that needs to affect
      --  I2C_Device_SO, which is initially False. On conclusion, we
      --  need to set it True.

      STM32.Board.Initialize_I2C_GPIO
        (STM32.I2C.I2C_Port (STM32.Board.MPU_Device.Port.all));
      STM32.Board.Configure_I2C
        (STM32.I2C.I2C_Port (STM32.Board.MPU_Device.Port.all));

      MPU9250_Init (STM32.Board.MPU_Device);
      MPU9250_Reset (STM32.Board.MPU_Device);

      --  Wait 50 ms after a reset has been performed
      Delay_After_Reset_Time :=
        Ada.Real_Time.Clock + Ada.Real_Time.Milliseconds (50);
      delay until Delay_After_Reset_Time;

      --  Activate MPU9250
      MPU9250_Set_Sleep_Enabled (STM32.Board.MPU_Device, False);
      --  Enable temp sensor
      MPU9250_Set_Temp_Sensor_Enabled (STM32.Board.MPU_Device, True);
      --  Disable interrupts
      MPU9250_Set_Int_Enabled (STM32.Board.MPU_Device, False);
      --  Connect the HMC5883L to the main I2C bus
      MPU9250_Set_I2C_Bypass_Enabled (STM32.Board.MPU_Device, True);
      --  Set x-axis gyro as clock source
      MPU9250_Set_Clock_Source (STM32.Board.MPU_Device, X_Gyro_Clk);
      --  Set gyro full-scale range
      MPU9250_Set_Full_Scale_Gyro_Range
        (STM32.Board.MPU_Device, Crazyflie_Config.IMU_GYRO_FS_CONFIG);
      --  Set accel full-scale range
      MPU9250_Set_Full_Scale_Accel_Range
        (STM32.Board.MPU_Device, Crazyflie_Config.IMU_ACCEL_FS_CONFIG);

      if DLPF_256Hz then
         --  256Hz digital low-pass filter only works with little vibrations
         --  Set output rate (15): 8000 / (1 + 15) = 500Hz
         MPU9250_Set_Rate (STM32.Board.MPU_Device, 15);
         MPU9250_Set_DLPF_Mode (STM32.Board.MPU_Device, MPU9250_DLPF_BW_256);
      else
         --  To low DLPF bandwidth might cause instability and decrease
         --  agility but it works well for handling vibrations and
         --  unbalanced propellers.
         --  Set output rate (1): 1000 / (1 + 1) = 500Hz
         MPU9250_Set_Rate (STM32.Board.MPU_Device, 1);
         --  Set digital low-pass bandwidth
         MPU9250_Set_DLPF_Mode (STM32.Board.MPU_Device, MPU9250_DLPF_BW_98);
      end if;

      if Use_Mag then
         AK8963.Initialize (STM32.Board.MAG_Device);
         if AK8963.Test_Connection (STM32.Board.MAG_Device) then
            Is_Magnetometer_Available := True;
            AK8963.Set_Mode (STM32.Board.MAG_Device,
                             AK8963.Continuous_2,
                             AK8963.Mode_16bit);
         end if;
      end if;

      Acc_Lp_Att_Factor := ACC_IIR_LPF_ATT_FACTOR;

      Cos_Pitch := Cos (0.0);
      Sin_Pitch := Sin (0.0);
      Cos_Roll := Cos (0.0);
      Sin_Roll := Sin (0.0);

      LPS25H_Sensor.Initialize;
      Is_Barometer_Available := LPS25H_Sensor.Is_Initialized;

      Is_Init := True;

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);

      Initialize_Parameter_Logging;
   end Init;

   --------------
   -- Test --
   --------------

   function Test return Boolean
   is
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);

      if not MPU9250_Test_Connection (STM32.Board.MPU_Device) then
         Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
         return False;
      end if;

      if not MPU9250_Self_Test
        (STM32.Board.MPU_Device, Console.Test, Console.Put_Line'Access)
      then
         Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
         return False;
      end if;

      if Is_Magnetometer_Available
        and then not AK8963.Self_Test (STM32.Board.MAG_Device)
      then
         Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
         return False;
      end if;

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
      return True;
   end Test;

   ------------------------------
   -- 6_Manufacturing_Test --
   ------------------------------

   function Manufacturing_Test_6 return Boolean is
      Test_Status : Boolean := False;
      Gyro_Out    : Gyroscope_Data;
      Acc_Out     : Accelerometer_Data;
      Pitch, Roll : Types.T_Degrees;
      Start_Time  : Ada.Real_Time.Time;
      use type Ada.Real_Time.Time;
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);

      Test_Status := MPU9250_Self_Test
        (STM32.Board.MPU_Device, Console.Test, Console.Put_Line'Access);
      Start_Time := Ada.Real_Time.Clock;

      if Test_Status then
         while Ada.Real_Time.Clock
           < Start_Time + VARIANCE_MAN_TEST_TIMEOUT loop
            Read_6 (Gyro_Out, Acc_Out);
            exit when Gyro_Bias.Is_Bias_Value_Found;
         end loop;

         if Gyro_Bias.Is_Bias_Value_Found then
            Pitch
              := Tan (-Acc_Out.X /
                        Sqrt (Acc_Out.Y * Acc_Out.Y + Acc_Out.Z * Acc_Out.Z)) *
                180.0 / Pi;
            Roll := Tan (Acc_Out.Y / Acc_Out.Z) * 180.0 * Pi;

            if abs (Roll) < MAN_TEST_LEVEL_MAX
              and abs (Pitch) < MAN_TEST_LEVEL_MAX
            then
               Test_Status := True;
            else
               Test_Status := False;
            end if;
         else
            Test_Status := False;
         end if;
      end if;

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
      return Test_Status;
   end Manufacturing_Test_6;

   ---------------------
   -- Calibrate_6 --
   ---------------------

   function Calibrate_6 return Boolean
   is
      Has_Found_Bias : Boolean;
      Next_Period    : Ada.Real_Time.Time;
      use type Ada.Real_Time.Time;
      Status         : Boolean;
   begin
      case Is_Calibrated is
         when Not_Calibrated =>
            null;
            --  Fall through
         when Calibrated =>
            return True;

         when Calibration_Error =>
            return False;
      end case;

      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);
      --  From now on, quit by setting Status as required and "goto Quit".

      if Is_Calibrated = Not_Calibrated then
         Next_Period := Ada.Real_Time.Clock + UPDATE_DT_MS;
         Variance_Sample_Time := Ada.Real_Time.Clock;

         loop
            MPU9250_Get_Motion_6 (STM32.Board.MPU_Device,
                                  Accel_IMU.Y, Accel_IMU.X, Accel_IMU.Z,
                                  Gyro_IMU.Y, Gyro_IMU.X, Gyro_IMU.Z);
            Add_Bias_Value (Gyro_Bias, Gyro_IMU);

            if Gyro_Bias.Is_Buffer_Filled then
               Find_Bias_Value (Gyro_Bias, Has_Found_Bias);

               if Has_Found_Bias then
                  --  TODO: led sequence to indicate that it is calibrated
                  Is_Calibrated := Calibrated;

                  Status := True;
                  goto Quit;
               else
                  Is_Calibrated := Calibration_Error;

                  Status := False;
                  goto Quit;
               end if;
            end if;

            Next_Period := Next_Period + UPDATE_DT_MS;
            delay until Next_Period;
         end loop;
      end if;

      <<Quit>>
      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
      return Status;
   end Calibrate_6;

   -----------------------
   -- Has_Barometer --
   -----------------------

   function Has_Barometer return Boolean is
   begin
      return Is_Barometer_Available;
   end Has_Barometer;

   ------------------------
   -- Read_6_Unprotected --
   ------------------------

   procedure Read_6_Unprotected
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data)
   is
   begin
      --  We invert X and Y because the chip is also inverted.
      MPU9250_Get_Motion_6 (STM32.Board.MPU_Device,
                            Accel_IMU.Y, Accel_IMU.X, Accel_IMU.Z,
                            Gyro_IMU.Y, Gyro_IMU.X, Gyro_IMU.Z);

      Acc_IRR_LP_Filter
        (Input         => Accel_IMU,
         Output        => Accel_LPF,
         Stored_Values => Accel_Stored_Values,
         Attenuation   => Types.T_Int32 (Acc_Lp_Att_Factor));
      Acc_Align_To_Gravity (Accel_LPF, Accel_LPF_Aligned);

      --  Re-map outputs
      Gyro.X :=
        -(Float (Gyro_IMU.X) - Gyro_Bias.Bias.X) * DEG_PER_LSB_CFG;
      Gyro.Y :=
        (Float (Gyro_IMU.Y) - Gyro_Bias.Bias.Y) * DEG_PER_LSB_CFG;
      Gyro.Z :=
        (Float (Gyro_IMU.Z) - Gyro_Bias.Bias.Z) * DEG_PER_LSB_CFG;

      Acc.X := (-Accel_LPF_Aligned.X) * G_PER_LSB_CFG;
      Acc.Y := Accel_LPF_Aligned.Y * G_PER_LSB_CFG;
      Acc.Z := Accel_LPF_Aligned.Z * G_PER_LSB_CFG;
   end Read_6_Unprotected;

   ----------------
   -- Read_6 --
   ----------------

   procedure Read_6
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data)
   is
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);

      Read_6_Unprotected (Gyro, Acc);

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
   end Read_6;

   ----------------
   -- Read_9 --
   ----------------

   procedure Read_9
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data;
      Mag  : in out Magnetometer_Data)
   is
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);

      Read_6_Unprotected (Gyro, Acc);
      if not Is_Magnetometer_Available then
         Mag := (others => 0.0);

      else
         declare
            Mx, My, Mz : AK8963.Gauss;
            Dead       : Boolean with Unreferenced;
         begin
            AK8963.Get_Heading (STM32.Board.MAG_Device, Mx, My, Mz);
            --  Get_Overflow_Status clears ST1 by reading ST2
            Dead := AK8963.Get_Overflow_Status (STM32.Board.MAG_Device);
            Mag  := (X => Mx,
                     Y => My,
                     Z => Mz);
         end;
      end if;

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
   end Read_9;

   -------------------------
   -- Read_Barometer_Data --
   -------------------------

   procedure Read_Barometer_Data
     (Press  :    out T_Pressure;
      Temp   :    out T_Temperature;
      Asl    :    out T_Altitude;
      Status :    out Boolean)
   is
      package Impl renames LPS25H;
      Press_I : Impl.Pressure;
      Temp_I : Impl.Temperature;
      Asl_I : Impl.Altitude;
   begin
      Ada.Synchronous_Task_Control.Suspend_Until_True (I2C_Device_SO);

      LPS25H_Sensor.Get_Data (Press_I, Temp_I, Asl_I, Status);
      if Status then
         Press := Press_I;
         Temp := Temp_I;
         Asl := Asl_I;
      end if;

      Ada.Synchronous_Task_Control.Set_True (I2C_Device_SO);
   end Read_Barometer_Data;

   ------------------------
   -- Add_Bias_Value --
   ------------------------

   procedure Add_Bias_Value
     (Bias_Obj : in out Bias_Object;
      Value    : Axis3_T_Int16) is
   begin
      Bias_Obj.Buffer (Bias_Obj.Buffer_Index) := Value;

      Bias_Obj.Buffer_Index := Bias_Obj.Buffer_Index + 1;

      if Bias_Obj.Buffer_Index > Bias_Obj.Buffer'Last then
         Bias_Obj.Is_Buffer_Filled := True;
         Bias_Obj.Buffer_Index := Bias_Obj.Buffer'First;
      end if;
   end Add_Bias_Value;

   -------------------------
   -- Find_Bias_Value --
   -------------------------

   procedure Find_Bias_Value
     (Bias_Obj       : in out Bias_Object;
      Has_Found_Bias : out Boolean) is
      use type Ada.Real_Time.Time;
   begin
      Has_Found_Bias := False;
      if Bias_Obj.Is_Buffer_Filled then
         declare
            Variance : Axis3_Float;
            Mean     : Axis3_Float;
         begin
            Calculate_Variance_And_Mean
              (Bias_Obj, Variance, Mean);

            if Variance.X < GYRO_VARIANCE_THRESHOLD_X and
              Variance.Y < GYRO_VARIANCE_THRESHOLD_Y and
              Variance.Z < GYRO_VARIANCE_THRESHOLD_Z and
              Ada.Real_Time.Clock >
              Variance_Sample_Time + GYRO_MIN_BIAS_TIMEOUT_MS
            then
               Variance_Sample_Time := Ada.Real_Time.Clock;
               Bias_Obj.Bias.X := Mean.X;
               Bias_Obj.Bias.Y := Mean.Y;
               Bias_Obj.Bias.Z := Mean.Z;
               Has_Found_Bias := True;
               Bias_Obj.Is_Bias_Value_Found := True;
            end if;
         end;
      end if;
   end Find_Bias_Value;

   -------------------------------------
   -- Calculate_Variance_And_Mean --
   -------------------------------------

   procedure Calculate_Variance_And_Mean
     (Bias_Obj : Bias_Object;
      Variance : out Axis3_Float;
      Mean     : out Axis3_Float)
   is
      Sum : Axis3_Float := (others => 0.0);

   begin
      for Value of Bias_Obj.Buffer loop
         Sum := Sum + Value;
      end loop;

      Mean := Sum / NBR_OF_BIAS_SAMPLES;

      Sum := (others => 0.0);

      for Value of Bias_Obj.Buffer loop
         Sum := Sum + (Mean - Value) ** 2;
      end loop;

      Variance := Sum / NBR_OF_BIAS_SAMPLES;
   end Calculate_Variance_And_Mean;

   ---------------------------
   -- Acc_IRR_LP_Filter --
   ---------------------------

   procedure Acc_IRR_LP_Filter
     (Input         : Axis3_T_Int16;
      Output        : out Axis3_T_Int32;
      Stored_Values : in out Axis3_T_Int32;
      Attenuation   : Types.T_Int32) is
   begin
      Filter.IIR_LP_Filter_Single
        (Types.T_Int32 (Input.X),
         Attenuation,
         Stored_Values.X,
         Output.X);
      Filter.IIR_LP_Filter_Single
        (Types.T_Int32 (Input.Y),
         Attenuation,
         Stored_Values.Y,
         Output.Y);
      Filter.IIR_LP_Filter_Single
        (Types.T_Int32 (Input.Z),
         Attenuation,
         Stored_Values.Z,
         Output.Z);
   end Acc_IRR_LP_Filter;

   ------------------------------
   -- Acc_Align_To_Gravity --
   ------------------------------

   procedure Acc_Align_To_Gravity
     (Input  : Axis3_T_Int32;
      Output : out Axis3_Float)
   is
      Input_F : constant Axis3_Float :=
        (Float (Input.X), Float (Input.Y), Float (Input.Z));
      Rx      : Axis3_Float;
      Ry      : Axis3_Float;
   begin
      --  Rotate around X-Axis
      Rx.X := Input_F.X;
      Rx.Y := Input_F.Y * Cos_Roll - Input_F.Z * Sin_Roll;
      Rx.Z := Input_F.Y * Sin_Roll + Input_F.Z * Cos_Roll;

      --  Rotate around Y-Axis
      Ry.X := Rx.X * Cos_Pitch - Rx.Z * Sin_Pitch;
      Ry.Y := Rx.Y;
      Ry.Z := -Rx.X * Sin_Pitch + Rx.Z * Cos_Pitch;

      Output.X := Ry.X;
      Output.Y := Ry.Y;
      Output.Z := Ry.Z;
   end Acc_Align_To_Gravity;

   ------------------------
   -- Initialize_Logging --
   ------------------------

   procedure Initialize_Parameter_Logging is
      Group_ID : Natural;
      use Parameter;
   begin
      Create_Parameter_Group (Name        => "imu_sensors",
                              Group_ID    => Group_ID);

      declare
         Parameter_Type : constant Parameter.Parameter_Variable_Type
           := (Size      => One_Byte,
               Floating  => False,
               Signed    => False,
               Read_Only => True,
               others    => <>);
      begin
         Parameter.Append_Parameter_Variable_To_Group
           (Group_ID,
            Name           => "HMC5883L",
            Parameter_Type => Parameter_Type,
            Variable       => Is_Magnetometer_Available'Address);
         Parameter.Append_Parameter_Variable_To_Group
           (Group_ID,
            Name           => "MS5611",  -- LPS25H would be better!
            Parameter_Type => Parameter_Type,
            Variable       => Is_Barometer_Available'Address);
      end;

   end Initialize_Parameter_Logging;

end IMU;
