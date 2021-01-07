------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
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

--  Previously in the spec
with Commander;
with Controller;
with Estimators;
with Estimators.Complementary;
with Estimators.Kalman;
with Free_Fall;
with IMU;
with Motors;
with Pid;
with Pid_Parameters;
with Power_Management;
with Stabilizer_Types;
pragma Elaborate_All (Pid); -- why?

with Config;
with Safety;
with Log;

package body Stabilizer
--  with SPARK_Mode,
--    Refined_State => (Stabilizer_State    => Is_Init,
--                      IMU_Outputs         => (Acc,
--                                              Gyro,
--                                              Mag),
--                      Actual_Angles       => (Euler_Roll_Actual,
--                                              Euler_Pitch_Actual,
--                                              Euler_Yaw_Actual),
--                      Desired_Angles      => (Euler_Roll_Desired,
--                                              Euler_Pitch_Desired,
--                                              Euler_Yaw_Desired),
--                      Desired_Rates       => (Roll_Rate_Desired,
--                                              Pitch_Rate_Desired,
--                                              Yaw_Rate_Desired),
--                      Command_Types       => (Roll_Type,
--                                              Pitch_Type,
--                                              Yaw_Type),
--                      Actuator_Commands   => (Actuator_Thrust,
--                                              Actuator_Roll,
--                                              Actuator_Pitch,
--                                              Actuator_Yaw),
--                      Motor_Powers        => (Motor_Power_M1,
--                                              Motor_Power_M2,
--                                              Motor_Power_M3,
--                                              Motor_Power_M4),
--                      V_Speed_Parameters  => (V_Speed_ASL_Fac,
--                                              V_Speed_Acc_Fac,
--                                              V_Acc_Deadband,
--                                              V_Speed_ASL_Deadband,
--                                              V_Speed_Limit,
--                                              V_Bias_Alpha),
--                      Asl_Parameters      => (Asl_Err_Deadband,
--                                              Asl_Alpha,
--                                              Asl_Alpha_Long),
--                      Alt_Hold_Parameters => (Alt_Hold_Err_Max,
--                                              Alt_Hold_Change_SENS,
--                                              Alt_Pid_Asl_Fac,
--                                              Alt_Pid_Alpha,
--                                              Alt_Hold_Base_Thrust,
--                                              Alt_Hold_Min_Thrust,
--                                              Alt_Hold_Max_Thrust),
--                      V_Speed_Variables   => (Acc_WZ,
--                                              Acc_MAG,
--                                              V_Speed,
--                                              V_Speed_Acc,
--                                              V_Speed_ASL),
--                      Asl_Variables       => (Temperature,
--                                              Pressure,
--                                              Asl,
--                                              Asl_Raw,
--                                              Asl_Long),
--                      Alt_Hold_Variables  => (Alt_Hold_PID,
--                                              Alt_Hold,
--                                              Set_Alt_Hold,
--                                              Alt_Hold_PID_Val,
--                                              Alt_Hold_Err,
--                                              Alt_Hold_Change,
--                                              Alt_Hold_Target))
is

   --  Instantiation of PID generic package for Altitude
   package Altitude_Pid is new Pid
     (INPUT_LOW_LIMIT   => IMU.T_Altitude'First,
      INPUT_HIGH_LIMIT  => IMU.T_Altitude'Last,
      OUTPUT_LOW_LIMIT  => Float'First / 8.0,
      OUTPUT_HIGH_LIMIT => Float'Last / 8.0,
      COEFF_LOW_LIMIT   => Pid_Parameters.MIN_ALTITUDE_COEFF,
      COEFF_HIGH_LIMIT  => Pid_Parameters.MAX_ALTITUDE_COEFF);

   --  Global variables and constants

   --  Defines in what divided update rate should the attitude
   --  control loop run relative the rate control loop.

   ATTITUDE_UPDATE_RATE_DIVIDER   : constant := 2;
   ATTITUDE_UPDATE_RATE_DIVIDER_F : constant := 2.0;
   --  500 Hz
   FUSION_UPDATE_DT : constant :=
     (1.0 / (IMU.UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER_F));

   --  500hz/5 = 100hz for barometer measurements
   ALTHOLD_UPDATE_RATE_DIVIDER   : constant := 5;
   ALTHOLD_UPDATE_RATE_DIVIDER_F : constant := 5.0;
   --  200 Hz
   ALTHOLD_UPDATE_DT : constant := (1.0 / (IMU.UPDATE_FREQ));

   Is_Init : Boolean := False;

   --  Estimator : Estimators.Complementary.Complementary_Estimator;
   Estimator : Estimators.Kalman.Kalman_Estimator;

   --  Sensor info (accelerometer, gyro, magnetometer, barometer).
   Sensor_Data : Stabilizer_Types.Sensor_Data;

   --  State info (attitude, position, velocity, acceleration).
   State_Data : Stabilizer_Types.State_Data;

   --  IMU outputs. The IMU is composed of an accelerometer, a gyroscope
   --  and a magnetometer.
   --  Gyro : IMU.Gyroscope_Data     := (0.0, 0.0, 0.0);
   --  Acc  : IMU.Accelerometer_Data := (0.0, 0.0, 0.0);
   --  Mag  : IMU.Magnetometer_Data  := (0.0, 0.0, 0.0);

   --  Actual angles. These angles are calculated by fusing
   --  accelerometer and gyro data in the Sensfusion algorithms.
   --  Euler_Roll_Actual   : Types.T_Degrees := 0.0;
   --  Euler_Pitch_Actual  : Types.T_Degrees := 0.0;
   --  Euler_Yaw_Actual    : Types.T_Degrees := 0.0;

   --  Desired angles. Obtained directly from the pilot.
   Euler_Roll_Desired  : Types.T_Degrees := 0.0;
   Euler_Pitch_Desired : Types.T_Degrees := 0.0;
   Euler_Yaw_Desired   : Types.T_Degrees := 0.0;

   --  Desired rates. Obtained directly from the pilot when
   --  commands are in RATE mode, or from the rate PIDs when commands
   --  are in ANGLE mode.
   Roll_Rate_Desired   : IMU.T_Rate  := 0.0;
   Pitch_Rate_Desired  : IMU.T_Rate  := 0.0;
   Yaw_Rate_Desired    : IMU.T_Rate  := 0.0;

   --  Variables used to calculate the altitude above sea level (ASL).
   Temperature  : IMU.T_Temperature := 0.0;
   Pressure     : IMU.T_Pressure    := 1000.0;
   Asl          : IMU.T_Altitude    := 0.0;
   Asl_Raw      : IMU.T_Altitude    := 0.0;
   Asl_Long     : IMU.T_Altitude    := 0.0;

   --  Variables used to calculate the vertical speed
   Acc_WZ       : Float   := 0.0;
   Acc_MAG      : Float   := 0.0;
   V_Speed_ASL  : Types.T_Speed := 0.0;
   V_Speed_Acc  : Types.T_Speed := 0.0;
   V_Speed      : Types.T_Speed := 0.0; --  Vertical speed (world frame)
                                        --  integrated from vertical
                                        --  acceleration.

   --  Variables used for the Altitude Hold mode.
   Alt_Hold_PID : Altitude_Pid.Object;
   --  Used for altitute hold mode.
   --  It gets reset when the bat ??? status changes.
   Alt_Hold     : Boolean := False;
   --  Currently in altitude hold mode.
   Set_Alt_Hold : Boolean := False;
   --  Hover mode just being activated.
   Alt_Hold_PID_Val : IMU.T_Altitude := 0.0;
   --  Output of the PID controller.
   Alt_Hold_Err     : Float := 0.0;
   --  Altitude error.
   Alt_Hold_Change  : IMU.T_Altitude := 0.0;
   --  Change in target altitude.
   Alt_Hold_Target  : IMU.T_Altitude := -1.0;
   --  Target altitude.

   --  Altitude hold & barometer params

   --  PID gain constants used everytime we reinitialise the PID controller.
   ALT_HOLD_KP          : constant := 0.5;
   ALT_HOLD_KI          : constant := 0.18;
   ALT_HOLD_KD          : constant := 0.0;

   --  Parameters used to calculate the vertical speed.
   V_Speed_ASL_Fac      : constant := 0.0;
   V_Speed_Acc_Fac      : constant := -48.0;
   V_Acc_Deadband       : constant := 0.05;
   --  Vertical acceleration deadband.
   V_Speed_ASL_Deadband : constant := 0.005;
   --  Vertical speed barometer deadband.
   V_Speed_Limit        : constant := 0.05;
   --  To saturate vertical velocity.
   V_Bias_Alpha         : constant := 0.98;
   --  Fusing factor used in ASL calc.

   --  Parameters used to calculate the altitude above see level (ASL).
   Asl_Err_Deadband     : constant := 0.00;
   --  error (target - altitude) deadband.
   Asl_Alpha            : constant := 0.92;
   --  Short term smoothing.
   Asl_Alpha_Long       : constant := 0.93;
   --  Long term smoothing.

   --  Parameters used for the Altitude Hold mode.
   Alt_Hold_Err_Max     : constant := 1.0;
   --  Max cap on current estimated altitude vs target altitude in
   --  meters.
   Alt_Hold_Change_SENS : constant := 200.0;
   --  Sensitivity of target altitude change (thrust input control)
   --  while hovering.  Lower = more sensitive & faster changes
   Alt_Pid_Asl_Fac          : constant := 13000.0;
   --  Relates meters asl to thrust.
   Alt_Pid_Alpha            : constant := 0.8;
   --  PID Smoothing.
   Alt_Hold_Min_Thrust      : constant := 00000;
   --  Minimum hover thrust.
   Alt_Hold_Base_Thrust     : constant := 43000;
   --  Approximate throttle needed when in perfect hover.  More weight
   --  / older battery can use a higher value.
   Alt_Hold_Max_Thrust  : constant := 60000;
   --  Max altitude hold thrust.

   --  Command variables used to control each angle.
   Roll_Type            : Commander.RPY_Type := Commander.ANGLE;
   Pitch_Type           : Commander.RPY_Type := Commander.ANGLE;
   Yaw_Type             : Commander.RPY_Type := Commander.RATE;

   --  Variables output from each rate PID, and from the Pilot (Thrust).
   Actuator_Thrust : Types.T_Uint16 := 0;
   Actuator_Roll   : Types.T_Int16  := 0;
   Actuator_Pitch  : Types.T_Int16  := 0;
   Actuator_Yaw    : Types.T_Int16  := 0;

   --  Variables used to control each motor's power.
   Motor_Power_M4  : Types.T_Uint16 := 0;
   Motor_Power_M2  : Types.T_Uint16 := 0;
   Motor_Power_M1  : Types.T_Uint16 := 0;
   Motor_Power_M3  : Types.T_Uint16 := 0;

   --  Body declarations

   procedure Init_Logging;

   --  Function called when Alt_Hold mode is activated. Holds the drone
   --  at a target altitude.
   procedure Alt_Hold_Update;

   --  Update the Attitude PIDs.
   procedure Update_Attitude;

   --  Update the Rate PIDs.
   procedure Update_Rate;

   --  Distribute power to the actuators with the PIDs outputs.
   procedure Distribute_Power
     (Thrust : Types.T_Uint16;
      Roll   : Types.T_Int16;
      Pitch  : Types.T_Int16;
      Yaw    : Types.T_Int16);

   --  Limit the given thrust to the maximum thrust supported by the motors.
   function Limit_Thrust (Value : Types.T_Int32) return Types.T_Uint16;
   pragma Inline (Limit_Thrust);

   --  Private procedures and functions

   ------------------
   -- Init_Logging --
   ------------------

   procedure Init_Logging
   is
      Dummy : Boolean;
   begin
      Log.Add_Variable (Group    => "motor",
                        Name     => "m1",
                        Typ => Log.UINT16,
                        Variable => Motor_Power_M1'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "motor",
                        Name     => "m2",
                        Typ => Log.UINT16,
                        Variable => Motor_Power_M2'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "motor",
                        Name     => "m3",
                        Typ => Log.UINT16,
                        Variable => Motor_Power_M3'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "motor",
                        Name     => "m4",
                        Typ => Log.UINT16,
                        Variable => Motor_Power_M4'Address,
                        Success  => Dummy);

      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "roll",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Att.Roll'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "pitch",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Att.Pitch'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "yaw",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Att.Yaw'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "thrust",
                        Typ      => Log.UINT16,
                        Variable => Actuator_Thrust'Address,
                        Success  => Dummy);

      Log.Add_Variable (Group    => "baro",
                        Name     => "asl",
                        Typ      => Log.FLOAT,
                        Variable => Asl'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "baro",
                        Name     => "temperature",
                        Typ      => Log.FLOAT,
                        Variable => Temperature'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "baro",
                        Name     => "pressure",
                        Typ      => Log.FLOAT,
                        Variable => Pressure'Address,
                        Success  => Dummy);

      Log.Add_Variable (Group    => "stateEstimate",
                        Name     => "x",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Pos.X'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stateEstimate",
                        Name     => "y",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Pos.Y'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stateEstimate",
                        Name     => "z",
                        Typ      => Log.FLOAT,
                        Variable => State_Data.Pos.Z'Address,
                        Success  => Dummy);
   end Init_Logging;

   ------------------
   -- Limit_Thrust --
   ------------------

   function Limit_Thrust (Value : Types.T_Int32) return Types.T_Uint16
   is
      use type Types.T_Int32;
      Res : Types.T_Uint16;
   begin
      if Value > Types.T_Int32 (Types.T_Uint16'Last) then
         Res := Types.T_Uint16'Last;
      elsif Value < 0 then
         Res := 0;
      else
         pragma Assert (Value <= Types.T_Int32 (Types.T_Uint16'Last));
         Res := Types.T_Uint16 (Value);
      end if;

      return Res;
   end Limit_Thrust;

   ----------------------
   -- Distribute_Power --
   ----------------------

   procedure Distribute_Power
     (Thrust : Types.T_Uint16;
      Roll   : Types.T_Int16;
      Pitch  : Types.T_Int16;
      Yaw    : Types.T_Int16)
   is
      use type Types.T_Int32;
      T : constant Types.T_Int32 := Types.T_Int32 (Thrust);
      R : Types.T_Int32 := Types.T_Int32 (Roll);
      P : Types.T_Int32 := Types.T_Int32 (Pitch);
      Y : constant Types.T_Int32 := Types.T_Int32 (Yaw);
   begin
      if Config.QUAD_FORMATION_X then
         R := R / 2;
         P := P / 2;

         Motor_Power_M1 := Limit_Thrust (T - R + P + Y);
         Motor_Power_M2 := Limit_Thrust (T - R - P - Y);
         Motor_Power_M3 := Limit_Thrust (T + R - P + Y);
         Motor_Power_M4 := Limit_Thrust (T + R + P - Y);
      else
         Motor_Power_M1 := Limit_Thrust (T + P + Y);
         Motor_Power_M2 := Limit_Thrust (T - R - Y);
         Motor_Power_M3 := Limit_Thrust (T - P + Y);
         Motor_Power_M4 := Limit_Thrust (T + R - Y);
      end if;

      Motors.Set_Power_With_Bat_Compensation (Motors.MOTOR_M1, Motor_Power_M1);
      Motors.Set_Power_With_Bat_Compensation (Motors.MOTOR_M2, Motor_Power_M2);
      Motors.Set_Power_With_Bat_Compensation (Motors.MOTOR_M3, Motor_Power_M3);
      Motors.Set_Power_With_Bat_Compensation (Motors.MOTOR_M4, Motor_Power_M4);
   end Distribute_Power;

   ---------------------
   -- Update_Attitude --
   ---------------------

   procedure Update_Attitude is
      Control : Stabilizer_Types.Control_Data;
   begin
      --  Not sure where Euler_*_Desired get set!
      Control.Roll  := Euler_Roll_Desired;
      Control.Pitch := Euler_Pitch_Desired;
      Control.Yaw   := Euler_Yaw_Desired;
      Commander.Get_Thrust (Thrust => Control.Thrust);

      Estimator.Estimate
        (State => State_Data,
         Sensors => Sensor_Data,
         Control => Control,
         Tick => 0);

      --  Vertical acceleration without gravity
      --  XXX should be able to get this from the state data.
      --  Acc_WZ := SensFusion6.Get_AccZ_Without_Gravity
      --    (Ax => Sensor_Data.Acc.X,
      --     Ay => Sensor_Data.Acc.Y,
      --     Az => Sensor_Data.Acc.Z);
      Acc_WZ := State_Data.Acc.Z;
      --  Magnitude of acceleration: not used
      --  Acc_MAG := (Acc.X * Acc.X) + (Acc.Y * Acc.Y) + (Acc.Z * Acc.Z);

      --  Estimate vertical speed from acceleration and Saturate
      --  it within a limit
      V_Speed := Safety.Saturate
        (V_Speed
           + Safety.Dead_Band (Acc_WZ, V_Acc_Deadband) * FUSION_UPDATE_DT,
         -V_Speed_Limit,
         V_Speed_Limit);

      --  Get the rate commands from the roll, pitch, yaw attitude PID's
      Controller.Correct_Attitude_Pid (State_Data.Att.Roll,
                                       State_Data.Att.Pitch,
                                       State_Data.Att.Yaw,
                                       Euler_Roll_Desired,
                                       Euler_Pitch_Desired,
                                       -Euler_Yaw_Desired);
      Controller.Get_Desired_Rate (Roll_Rate_Desired,
                                   Pitch_Rate_Desired,
                                   Yaw_Rate_Desired);
   end Update_Attitude;

   -----------------
   -- Update_Rate --
   -----------------

   procedure Update_Rate is
      use type Commander.RPY_Type;
   begin
      --  If CF is in Rate mode, give the angles given by the pilot
      --  as input for the Rate PIDs
      if Roll_Type = Commander.RATE then
         Roll_Rate_Desired := Euler_Roll_Desired;
      end if;

      if Pitch_Type = Commander.RATE then
         Pitch_Rate_Desired := Euler_Pitch_Desired;
      end if;

      if Yaw_Type = Commander.RATE then
         Yaw_Rate_Desired := -Euler_Yaw_Desired;
      end if;

      Controller.Correct_Rate_PID (Sensor_Data.Gyro.X,
                                   -Sensor_Data.Gyro.Y,
                                   Sensor_Data.Gyro.Z,
                                   Roll_Rate_Desired,
                                   Pitch_Rate_Desired,
                                   Yaw_Rate_Desired);
      Controller.Get_Actuator_Output (Actuator_Roll,
                                      Actuator_Pitch,
                                      Actuator_Yaw);
   end Update_Rate;

   ---------------------
   -- Alt_Hold_Update --
   ---------------------

   procedure Alt_Hold_Update
   is
      Barometer_Data_Valid   : Boolean;
      Prev_Integ          : Float;
      Baro_V_Speed        : Types.T_Speed;
      Alt_Hold_PID_Out    : IMU.T_Altitude;
      Raw_Thrust          : Types.T_Int16;
      use type Types.T_Int32;
   begin
      --  Get altitude hold commands from the pilot
      Commander.Get_Alt_Hold (Alt_Hold, Set_Alt_Hold, Alt_Hold_Change);

      --  Get barometer altitude estimations
      IMU.Read_Barometer_Data
        (Pressure, Temperature, Asl_Raw, Barometer_Data_Valid);
      if Barometer_Data_Valid then
         Asl := Safety.Saturate
           (Asl * Asl_Alpha + Asl_Raw * (1.0 - Asl_Alpha),
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);
         Asl_Long := Safety.Saturate
           (Asl_Long * Asl_Alpha_Long + Asl_Raw * (1.0 - Asl_Alpha_Long),
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);
      end if;

      --  Estimate vertical speed based on successive barometer readings
      V_Speed_ASL := Safety.Saturate
        (Safety.Dead_Band (Asl - Asl_Long, V_Speed_ASL_Deadband),
         -V_Speed_Limit,
         V_Speed_Limit);
      --  Estimate vertical speed based on Acc - fused with baro
      --  to reduce drift
      V_Speed := Safety.Saturate
        (V_Speed * V_Bias_Alpha + V_Speed_ASL * (1.0 - V_Bias_Alpha),
         -V_Speed_Limit,
         V_Speed_Limit);
      V_Speed_Acc := V_Speed;

      --  Reset Integral gain of PID controller if being charged
      if not Power_Management.Is_Discharging then
         Altitude_Pid.Set_Integral_Term (Alt_Hold_PID, 0.0);
      end if;

      --  Altitude hold mode just activated, set target altitude as current
      --  altitude. Reuse previous integral term as a starting point
      if Set_Alt_Hold then
         --  Set target altitude to current altitude
         Alt_Hold_Target := Asl;
         --  Cache last integral term for reuse after PID init
         Prev_Integ := Altitude_Pid.Get_Integral_Term (Alt_Hold_PID);

         --  Reset PID controller
         Altitude_Pid.Init (Alt_Hold_PID,
                            Asl,
                            ALT_HOLD_KP,
                            ALT_HOLD_KI,
                            ALT_HOLD_KD,
                            -Pid_Parameters.DEFAULT_INTEGRATION_LIMIT,
                            Pid_Parameters.DEFAULT_INTEGRATION_LIMIT,
                            ALTHOLD_UPDATE_DT);

         Altitude_Pid.Set_Integral_Term (Alt_Hold_PID, Prev_Integ);

         Altitude_Pid.Update (Alt_Hold_PID, Asl, False);
         Alt_Hold_PID_Val := Safety.Saturate
           (Altitude_Pid.Get_Output (Alt_Hold_PID),
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);
      end if;

      if Alt_Hold then
         --  Update the target altitude and the PID
         Alt_Hold_Target := Safety.Saturate
           (Alt_Hold_Target + Alt_Hold_Change / Alt_Hold_Change_SENS,
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);
         Altitude_Pid.Set_Desired (Alt_Hold_PID, Alt_Hold_Target);

         --  Compute error (current - target), limit the error
         Alt_Hold_Err := Safety.Saturate
           (Safety.Dead_Band (Asl - Alt_Hold_Target, Asl_Err_Deadband),
            -Alt_Hold_Err_Max,
            Alt_Hold_Err_Max);

         --  Update the Altitude PID
         Altitude_Pid.Set_Error (Alt_Hold_PID, -Alt_Hold_Err);
         Altitude_Pid.Update (Alt_Hold_PID, Asl, False);

         Baro_V_Speed := Safety.Saturate
           ((1.0 - Alt_Pid_Alpha) * ((V_Speed_Acc * V_Speed_Acc_Fac)
                                       + (V_Speed_ASL * V_Speed_ASL_Fac)),
            Types.T_Speed'First,
            Types.T_Speed'Last);
         Alt_Hold_PID_Out := Safety.Saturate
           (Altitude_Pid.Get_Output (Alt_Hold_PID),
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);

         Alt_Hold_PID_Val := Safety.Saturate
           (Alt_Pid_Alpha * Alt_Hold_PID_Val + Baro_V_Speed + Alt_Hold_PID_Out,
            IMU.T_Altitude'First,
            IMU.T_Altitude'Last);

         Raw_Thrust := Safety.Truncate_To_T_Int16
           (Alt_Hold_PID_Val * Alt_Pid_Asl_Fac);
         Actuator_Thrust := Safety.Saturate
           (Limit_Thrust (Types.T_Int32 (Raw_Thrust)
                            + Types.T_Int32 (Alt_Hold_Base_Thrust)),
            Alt_Hold_Min_Thrust,
            Alt_Hold_Max_Thrust);
      end if;

   end Alt_Hold_Update;

   --  Public functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      if Is_Init then
         return;
      end if;

      Controller.Init;

      Init_Logging;

      Estimator.Initialize;

      Is_Init := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Init;
   end Test;

   ------------------
   -- Control_Loop --
   ------------------

   procedure Control_Loop
     (Attitude_Update_Counter : in out Types.T_Uint32;
      Alt_Hold_Update_Counter : in out Types.T_Uint32)
   is
      use type Types.T_Int16;
      use type Types.T_Uint16;
      use type Types.T_Uint32;
   begin
      --  Original code does this every other call (i.e. 250 Hz, calls
      --  at 500 Hz).
      Update_Attitude;

      --  Increment the counters
      Attitude_Update_Counter := Attitude_Update_Counter + 1;
      Alt_Hold_Update_Counter := Alt_Hold_Update_Counter + 1;

      --  Check if the drone is in Free fall or has landed.
      --  This check is enabled by default, but can be disabled
      Free_Fall.FF_Check_Event (IMU.Accelerometer_Data'
                                  (X => Sensor_Data.Acc.X,
                                   Y => Sensor_Data.Acc.Y,
                                   Z => Sensor_Data.Acc.Z));

      --  Get commands from the pilot
      Commander.Get_RPY (Euler_Roll_Desired,
                         Euler_Pitch_Desired,
                         Euler_Yaw_Desired);

      Commander.Get_RPY_Type (Roll_Type, Pitch_Type, Yaw_Type);

      --  If FF checks are enabled and the drone is in recovery,
      --  override the pilot commands
      Free_Fall.FF_Get_Recovery_Commands (Euler_Roll_Desired,
                                          Euler_Pitch_Desired,
                                          Roll_Type,
                                          Pitch_Type);

      --  --  Update attitude at IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER
      --  --  By default the result is 250 Hz
      --  if Attitude_Update_Counter >= ATTITUDE_UPDATE_RATE_DIVIDER then
      --     --  Update attitude
      --     Update_Attitude;
      --     --  Reset the counter
      --     Attitude_Update_Counter := 0;
      --  end if;

      if IMU.Has_Barometer and
        Alt_Hold_Update_Counter >= ALTHOLD_UPDATE_RATE_DIVIDER
      then
         --  Altidude hold mode update
         Alt_Hold_Update;
         --  Reset the counter
         Alt_Hold_Update_Counter := 0;
      end if;

      Update_Rate;

      if not Alt_Hold or not IMU.Has_Barometer then
         --  Get thrust from the commander if alt hold mode
         --  not activated
         Commander.Get_Thrust (Actuator_Thrust);
         --  Override the thrust if the drone is in freefall
         Free_Fall.FF_Get_Recovery_Thrust (Actuator_Thrust);
      else
         --  Added so thrust can be set to 0 while in altitude hold mode
         --  after disconnect
         Commander.Watchdog;
      end if;

      if Actuator_Thrust > 0 then
         --  Ensure that there is no overflow when changing Yaw sign
         if Actuator_Yaw = Types.T_Int16'First then
            Actuator_Yaw := -Types.T_Int16'Last;
         end if;

         Distribute_Power (Actuator_Thrust, Actuator_Roll,
                           Actuator_Pitch, -Actuator_Yaw);
      else
         Distribute_Power (0, 0, 0, 0);
         Controller.Reset_All_Pid;
      end if;
   end Control_Loop;

begin
   Estimators.Set_Required_Estimator (To => Estimator);
end Stabilizer;
