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

with Ada.Numerics;  use Ada.Numerics;
with Ada.Real_Time;

with Filter;
with Types;
with MPU9250;       use MPU9250;  --  in Ada Drivers Library

package IMU is

   --  These ranges are deduced from the MPU9150 specification.
   --  It corresponds to the maximum range of values that can be output
   --  by the IMU.

   --  Type for angular speed output from gyro, degrees/s.
   subtype T_Rate is Float range -100_000.0  .. 100_000.0;
   --  Type for angular speed output from gyro, rad/s.
   subtype T_Rate_Rad
     is Float range T_Rate'First * Pi / 180.0 .. T_Rate'Last * Pi / 180.0;
   --  Type for acceleration output from accelerometer, in G.
   subtype T_Acc  is Float range -16.0 .. 16.0;
   --  Type for magnetometer output, in micro-Teslas.
   subtype T_Mag  is Float range -1_200.0  .. 1_200.0;

   --  Type used when we want to collect several accelerometer samples.
   type T_Acc_Array is array (Integer range <>) of T_Acc;

   --  Type used to ensure that accelation normalization can't lead to a
   --  division by zero in SensFusion6 algorithms.
   MIN_NON_ZERO_ACC : constant := 2.0 ** (-74);

   subtype T_Acc_Lifted is T_Acc; -- with
   --         Static_Predicate => T_Acc_Lifted = 0.0 or else
   --         T_Acc_Lifted not in -MIN_NON_ZERO_ACC .. MIN_NON_ZERO_ACC;

   --  Type used to represent gyroscope data
   --  along each axis (X, Y, Z).
   type Gyroscope_Data is record
      X : T_Rate;
      Y : T_Rate;
      Z : T_Rate;
   end record;

   --  Type used to represent accelerometer data
   --  along each axis (X, Y, Z).
   type Accelerometer_Data is record
      X : T_Acc;
      Y : T_Acc;
      Z : T_Acc;
   end record;

   --  Type used to represent magnetometer data
   --  along each axis (X, Y, Z).
   type Magnetometer_Data is record
      X : T_Mag;
      Y : T_Mag;
      Z : T_Mag;
   end record;

   --  Types for barometric data.
   subtype T_Pressure is Float range 450.0 .. 1100.0;   -- in mBar
   subtype T_Temperature is Float range -20.0 .. 80.0;  -- in degrees Celsius
   subtype T_Altitude is Float range -8000.0 .. 8000.0; -- m above sea level

   --  Global variables and constants

   UPDATE_FREQ  : constant := 250.0;
   UPDATE_DT    : constant := 1.0 / UPDATE_FREQ;
   UPDATE_DT_MS : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (4);

   --  Number of samples used for bias calculation
   NBR_OF_BIAS_SAMPLES      : constant := 1024;
   GYRO_MIN_BIAS_TIMEOUT_MS : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (1_000);

   --  Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
   --  The highest cut-off freq that will have any affect is fs /(2*pi).
   --  E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
   ACC_WANTED_LPF_CUTOFF_HZ : constant := 4.0;
   --  Attenuation should be between 1 to 256.
   --  F0 = fs / 2*pi*attenuation ->
   --  Attenuation = fs / 2*pi*f0
   ACC_IIR_LPF_ATTENUATION  : constant Float
     := Float (UPDATE_FREQ) / (2.0 * Pi * ACC_WANTED_LPF_CUTOFF_HZ);
   ACC_IIR_LPF_ATT_FACTOR   : constant Types.T_Uint8
     := Types.T_Uint8 (Float (2 ** Filter.IIR_SHIFT)
                 / ACC_IIR_LPF_ATTENUATION + 0.5);

   GYRO_VARIANCE_BASE        : constant := 2_000.0;
   GYRO_VARIANCE_THRESHOLD_X : constant := (GYRO_VARIANCE_BASE);
   GYRO_VARIANCE_THRESHOLD_Y : constant := (GYRO_VARIANCE_BASE);
   GYRO_VARIANCE_THRESHOLD_Z : constant := (GYRO_VARIANCE_BASE);

   DEG_PER_LSB_CFG       : constant := MPU9250_DEG_PER_LSB_2000;
   G_PER_LSB_CFG         : constant := MPU9250_G_PER_LSB_8;

   VARIANCE_MAN_TEST_TIMEOUT : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (1_000);
   MAN_TEST_LEVEL_MAX : constant := 5.0;

   --  Procedures and functions

   --  Initialize the IMU device/
   procedure Init (Use_Mag    : Boolean;
                   DLPF_256Hz : Boolean);

   --  Test if the IMU device is initialized.
   function Test return Boolean;

   --  Manufacturing test to ensure that IMU is not broken.
   function Manufacturing_Test_6 return Boolean;

   --  Read gyro and accelerometer measurements from the IMU.
   procedure Read_6
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data);

   --  Read gyro, accelerometer and magnetometer measurements from the IMU.
   procedure Read_9
     (Gyro : in out Gyroscope_Data;
      Acc  : in out Accelerometer_Data;
      Mag  : in out Magnetometer_Data);

   --  Calibrates the IMU. Returns True if successful, False otherwise.
   function Calibrate_6 return Boolean;

   --  Return True if the IMU has an initialized barometer, False otherwise.
   function Has_Barometer return Boolean;

   procedure Read_Barometer_Data
     (Press  :    out T_Pressure;
      Temp   :    out T_Temperature;
      Asl    :    out T_Altitude;
      Status :    out Boolean)
   with Pre => Has_Barometer;

private
   --  Types

   type Axis3_T_Int16 is record
      X : Types.T_Int16 := 0;
      Y : Types.T_Int16 := 0;
      Z : Types.T_Int16 := 0;
   end record;

   type Axis3_T_Int32 is record
      X : Types.T_Int32 := 0;
      Y : Types.T_Int32 := 0;
      Z : Types.T_Int32 := 0;
   end record;

   type Axis3_Float is record
      X : Float := 0.0;
      Y : Float := 0.0;
      Z : Float := 0.0;
   end record;

   function "+" (A1 : Axis3_Float; A2 : Axis3_Float) return Axis3_Float
   is ((X => A1.X + A2.X,
        Y => A1.Y + A2.Y,
        Z => A1.Z + A2.Z));

   function "+" (AF : Axis3_Float; AI : Axis3_T_Int16) return Axis3_Float
   is ((X => AF.X + Float (AI.X),
        Y => AF.Y + Float (AI.Y),
        Z => AF.Z + Float (AI.Z)));

   function "**" (AF : Axis3_Float; E : Integer) return Axis3_Float
   is ((X => AF.X ** E,
        Y => AF.Y ** E,
        Z => AF.Z ** E));

   function "-" (AF : Axis3_Float; AI : Axis3_T_Int16) return Axis3_Float
   is ((X => AF.X - Float (AI.X),
        Y => AF.Y - Float (AI.Y),
        Z => AF.Z - Float (AI.Z)));

   function "/" (AF : Axis3_Float; Val : Integer) return Axis3_Float
   is ((X => AF.X / Float (Val),
        Y => AF.Y / Float (Val),
        Z => AF.Z / Float (Val)));

   type Bias_Buffer_Array is
     array (1 .. NBR_OF_BIAS_SAMPLES) of Axis3_T_Int16;

   --  Type used for bias calculation
   type Bias_Object is record
      Bias                : Axis3_Float;
      Buffer              : Bias_Buffer_Array;
      Buffer_Index        : Positive := Bias_Buffer_Array'First;
      Is_Bias_Value_Found : Boolean  := False;
      Is_Buffer_Filled    : Boolean  := False;
   end record;

   --  Global variables and constants

   Is_Init : Boolean := False;

   type Calibration_Status is
     (Not_Calibrated,
      Calibrated,
      Calibration_Error);

   --  Barometer not available for now.
   --  TODO: add the code to manipulate it
   Is_Barometer_Available    : Boolean := False;
   Is_Magnetometer_Available : Boolean := False;
   Is_Calibrated             : Calibration_Status := Not_Calibrated;

   Variance_Sample_Time  : Ada.Real_Time.Time;
   Acc_Lp_Att_Factor : Types.T_Uint8;

   --  Raw values retrieved from IMU
   Accel_IMU           : Axis3_T_Int16;
   Gyro_IMU            : Axis3_T_Int16;
   --  Acceleration after applying the IIR LPF filter
   Accel_LPF           : Axis3_T_Int32;
   --  Use to store the IIR LPF filter feedback
   Accel_Stored_Values : Axis3_T_Int32;
   --  Acceleration after aligning with gravity
   Accel_LPF_Aligned   : Axis3_Float;

   Cos_Pitch : Float;
   Sin_Pitch : Float;
   Cos_Roll  : Float;
   Sin_Roll  : Float;

   --  Bias objects used for bias calculation
   Gyro_Bias : Bias_Object;

   --  Procedures and functions

   --  Add a new value to the variance buffer and if it is full
   --  replace the oldest one. Thus a circular buffer.
   procedure Add_Bias_Value
     (Bias_Obj : in out Bias_Object;
      Value    : Axis3_T_Int16);

   --  Check if the variances is below the predefined thresholds.
   --  The bias value should have been added before calling this.
   procedure Find_Bias_Value
     (Bias_Obj       : in out Bias_Object;
      Has_Found_Bias : out Boolean);

   --  Calculate the variance and mean for the bias buffer.
   procedure Calculate_Variance_And_Mean
     (Bias_Obj : Bias_Object;
      Variance : out Axis3_Float;
      Mean     : out Axis3_Float);

   --  Apply IIR LP Filter on each axis.
   procedure Acc_IRR_LP_Filter
     (Input         : Axis3_T_Int16;
      Output        : out Axis3_T_Int32;
      Stored_Values : in out Axis3_T_Int32;
      Attenuation   : Types.T_Int32);

   --  Compensate for a misaligned accelerometer. It uses the trim
   --  data gathered from the UI and written in the config-block to
   --  rotate the accelerometer to be aligned with gravity.
   procedure Acc_Align_To_Gravity
     (Input  : Axis3_T_Int32;
      Output : out Axis3_Float);

end IMU;
