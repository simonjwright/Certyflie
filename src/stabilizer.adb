------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
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

with Config;
with Safety;
with Log;

package body Stabilizer
with SPARK_Mode,
  Refined_State => (Stabilizer_State    => Is_Init,
                    IMU_Outputs         => (Acc,
                                            Gyro,
                                            Mag),
                    Actual_Angles       => (Euler_Roll_Actual,
                                            Euler_Pitch_Actual,
                                            Euler_Yaw_Actual),
                    Desired_Angles      => (Euler_Roll_Desired,
                                            Euler_Pitch_Desired,
                                            Euler_Yaw_Desired),
                    Desired_Rates       => (Roll_Rate_Desired,
                                            Pitch_Rate_Desired,
                                            Yaw_Rate_Desired),
                    Command_Types       => (Roll_Type,
                                            Pitch_Type,
                                            Yaw_Type),
                    Actuator_Commands   => (Actuator_Thrust,
                                            Actuator_Roll,
                                            Actuator_Pitch,
                                            Actuator_Yaw),
                    Motor_Powers        => (Motor_Power_M1,
                                            Motor_Power_M2,
                                            Motor_Power_M3,
                                            Motor_Power_M4),
                    V_Speed_Parameters  => (V_Speed_ASL_Fac,
                                            V_Speed_Acc_Fac,
                                            V_Acc_Deadband,
                                            V_Speed_ASL_Deadband,
                                            V_Speed_Limit,
                                            V_Bias_Alpha),
                    Asl_Parameters      => (Asl_Err_Deadband,
                                            Asl_Alpha,
                                            Asl_Alpha_Long),
                    Alt_Hold_Parameters => (Alt_Hold_Err_Max,
                                            Alt_Hold_Change_SENS,
                                            Alt_Pid_Asl_Fac,
                                            Alt_Pid_Alpha,
                                            Alt_Hold_Base_Thrust,
                                            Alt_Hold_Min_Thrust,
                                            Alt_Hold_Max_Thrust),
                    V_Speed_Variables   => (Acc_WZ,
                                            Acc_MAG,
                                            V_Speed,
                                            V_Speed_Acc,
                                            V_Speed_ASL),
                    Asl_Variables       => (Temperature,
                                            Pressure,
                                            Asl,
                                            Asl_Raw,
                                            Asl_Long),
                    Alt_Hold_Variables  => (Alt_Hold_PID,
                                            Alt_Hold,
                                            Set_Alt_Hold,
                                            Alt_Hold_PID_Val,
                                            Alt_Hold_Err,
                                            Alt_Hold_Change,
                                            Alt_Hold_Target))
is

   --  Body declarations

   procedure Init_Logging;

   --  Private procedures and functions

   ------------------
   -- Init_Logging --
   ------------------

   procedure Init_Logging
     with SPARK_Mode => Off -- 'Address isn't allowed in SPARK
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
                        Variable => Euler_Roll_Actual'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "pitch",
                        Typ      => Log.FLOAT,
                        Variable => Euler_Pitch_Actual'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "yaw",
                        Typ      => Log.FLOAT,
                        Variable => Euler_Yaw_Actual'Address,
                        Success  => Dummy);
      Log.Add_Variable (Group    => "stabilizer",
                        Name     => "thrust",
                        Typ      => Log.UINT16,
                        Variable => Actuator_Thrust'Address,
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
   begin
      SensFusion6.Update_Q (Gyro.X, Gyro.Y, Gyro.Z,
                            Acc.X, Acc.Y, Acc.Z,
                            Mag.X, Mag.Y, Mag.Z,
                            FUSION_UPDATE_DT);
      --  Get Euler angles
      SensFusion6.Get_Euler_RPY (Euler_Roll_Actual,
                                 Euler_Pitch_Actual,
                                 Euler_Yaw_Actual);
      --  Vertical acceleration woithout gravity
      Acc_WZ := SensFusion6.Get_AccZ_Without_Gravity (Acc.X, Acc.Y, Acc.Z);
      Acc_MAG := (Acc.X * Acc.X) + (Acc.Y * Acc.Y) + (Acc.Z * Acc.Z);

      --  Estimate vertical speed from acceleration and Saturate
      --  it within a limit
      V_Speed := Safety.Saturate
        (V_Speed
         + Safety.Dead_Band (Acc_WZ, V_Acc_Deadband) * FUSION_UPDATE_DT,
         -V_Speed_Limit,
         V_Speed_Limit);

      --  Get the rate commands from the roll, pitch, yaw attitude PID's
      Controller.Correct_Attitude_Pid (Euler_Roll_Actual,
                                       Euler_Pitch_Actual,
                                       Euler_Yaw_Actual,
                                       Euler_Roll_Desired,
                                       Euler_Pitch_Desired,
                                       -Euler_Yaw_Desired);
      Controller.Get_Desired_Rate (Roll_Rate_Desired, Pitch_Rate_Desired,
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

      Controller.Correct_Rate_PID (Gyro.X, -Gyro.Y, Gyro.Z,
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
      LPS25H_Data_Valid   : Boolean;
      Prev_Integ          : Float;
      Baro_V_Speed        : Types.T_Speed;
      Alt_Hold_PID_Out    : LPS25h.T_Altitude;
      Raw_Thrust          : Types.T_Int16;
      use type Types.T_Int32;
   begin
      --  Get altitude hold commands from the pilot
      Commander.Get_Alt_Hold (Alt_Hold, Set_Alt_Hold, Alt_Hold_Change);

      --  Get barometer altitude estimations
      LPS25h.LPS25h_Get_Data
        (Pressure, Temperature, Asl_Raw, LPS25H_Data_Valid);
      if LPS25H_Data_Valid then
         Asl := Safety.Saturate
           (Asl * Asl_Alpha + Asl_Raw * (1.0 - Asl_Alpha),
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);
         Asl_Long := Safety.Saturate
           (Asl_Long * Asl_Alpha_Long + Asl_Raw * (1.0 - Asl_Alpha_Long),
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);
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
         Alt_Hold_PID.Integ := 0.0;
      end if;

      --  Altitude hold mode just activated, set target altitude as current
      --  altitude. Reuse previous integral term as a starting point
      if Set_Alt_Hold then
         --  Set target altitude to current altitude
         Alt_Hold_Target := Asl;
         --  Cache last integral term for reuse after PID init
         Prev_Integ := Alt_Hold_PID.Integ;

         --  Reset PID controller
         Altitude_Pid.Init (Alt_Hold_PID,
                            Asl,
                            ALT_HOLD_KP,
                            ALT_HOLD_KP,
                            ALT_HOLD_KD,
                            -Pid_Parameters.DEFAULT_INTEGRATION_LIMIT,
                            Pid_Parameters.DEFAULT_INTEGRATION_LIMIT,
                            ALTHOLD_UPDATE_DT);

         Alt_Hold_PID.Integ := Prev_Integ;

         Altitude_Pid.Update (Alt_Hold_PID, Asl, False);
         Alt_Hold_PID_Val := Safety.Saturate
           (Altitude_Pid.Get_Output (Alt_Hold_PID),
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);
      end if;

      if Alt_Hold then
         --  Update the target altitude and the PID
         Alt_Hold_Target := Safety.Saturate
           (Alt_Hold_Target + Alt_Hold_Change / Alt_Hold_Change_SENS,
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);
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
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);

         Alt_Hold_PID_Val := Safety.Saturate
           (Alt_Pid_Alpha * Alt_Hold_PID_Val + Baro_V_Speed + Alt_Hold_PID_Out,
            LPS25h.T_Altitude'First,
            LPS25h.T_Altitude'Last);

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
      --  Magnetometer not used for the moment
      IMU.Read_9 (Gyro, Acc, Mag);

      --  Increment the counters
      Attitude_Update_Counter := Attitude_Update_Counter + 1;
      Alt_Hold_Update_Counter := Alt_Hold_Update_Counter + 1;

      --  Check if the drone is in Free fall or has landed.
      --  This check is enabled by default, but can be disabled
      Free_Fall.FF_Check_Event (Acc);

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

      --  Update attitude at IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER
      --  By default the result is 250 Hz
      if Attitude_Update_Counter >= ATTITUDE_UPDATE_RATE_DIVIDER then
         --  Update attitude
         Update_Attitude;
         --  Reset the counter
         Attitude_Update_Counter := 0;
      end if;

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

end Stabilizer;
