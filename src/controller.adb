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

with Safety;

package body Controller
with
  SPARK_Mode,
  Refined_State => (Attitude_PIDs     => (Roll_Pid,
                                          Pitch_Pid,
                                          Yaw_Pid),
                    Rate_PIDs         => (Roll_Rate_Pid,
                                          Pitch_Rate_Pid,
                                          Yaw_Rate_Pid),
                    Controller_State  =>  Is_Init)
is

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      Rate_Pid.Init (Roll_Rate_Pid, 0.0,
                     Pid_Parameters.ROLL_RATE_KP,
                     Pid_Parameters.ROLL_RATE_KI,
                     Pid_Parameters.ROLL_RATE_KD,
                     -Pid_Parameters.ROLL_RATE_INTEGRATION_LIMIT,
                     Pid_Parameters.ROLL_RATE_INTEGRATION_LIMIT,
                     IMU.UPDATE_DT);
      Rate_Pid.Init (Pitch_Rate_Pid,
                     0.0,
                     Pid_Parameters.PITCH_RATE_KP,
                     Pid_Parameters.PITCH_RATE_KI,
                     Pid_Parameters.PITCH_RATE_KD,
                     -Pid_Parameters.PITCH_RATE_INTEGRATION_LIMIT,
                     Pid_Parameters.PITCH_RATE_INTEGRATION_LIMIT,
                     IMU.UPDATE_DT);
      Rate_Pid.Init (Yaw_Rate_Pid,
                     0.0,
                     Pid_Parameters.YAW_RATE_KP,
                     Pid_Parameters.YAW_RATE_KI,
                     Pid_Parameters.YAW_RATE_KD,
                     -Pid_Parameters.YAW_RATE_INTEGRATION_LIMIT,
                     Pid_Parameters.YAW_RATE_INTEGRATION_LIMIT,
                     IMU.UPDATE_DT);

      Attitude_Pid.Init (Roll_Pid,
                         0.0,
                         Pid_Parameters.ROLL_KP,
                         Pid_Parameters.ROLL_KI,
                         Pid_Parameters.ROLL_KD,
                         -Pid_Parameters.ROLL_INTEGRATION_LIMIT,
                         Pid_Parameters.ROLL_INTEGRATION_LIMIT,
                         IMU.UPDATE_DT);
      Attitude_Pid.Init (Pitch_Pid,
                         0.0,
                         Pid_Parameters.PITCH_KP,
                         Pid_Parameters.PITCH_KI,
                         Pid_Parameters.PITCH_KD,
                         -Pid_Parameters.PITCH_INTEGRATION_LIMIT,
                         Pid_Parameters.PITCH_INTEGRATION_LIMIT,
                         IMU.UPDATE_DT);
      Attitude_Pid.Init (Yaw_Pid,
                         0.0,
                         Pid_Parameters.YAW_KP,
                         Pid_Parameters.YAW_KI,
                         Pid_Parameters.YAW_KD,
                         -Pid_Parameters.YAW_INTEGRATION_LIMIT,
                         Pid_Parameters.YAW_INTEGRATION_LIMIT,
                         IMU.UPDATE_DT);

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
   -- Correct_Rate_PID --
   ----------------------

   procedure Correct_Rate_PID
     (Roll_Rate_Actual   : IMU.T_Rate;
      Pitch_Rate_Actual  : IMU.T_Rate;
      Yaw_Rate_Actual    : IMU.T_Rate;
      Roll_Rate_Desired  : IMU.T_Rate;
      Pitch_Rate_Desired : IMU.T_Rate;
      Yaw_Rate_Desired   : IMU.T_Rate) is
   begin
      Rate_Pid.Set_Desired (Roll_Rate_Pid, Roll_Rate_Desired);
      Rate_Pid.Set_Desired (Pitch_Rate_Pid, Pitch_Rate_Desired);
      Rate_Pid.Set_Desired (Yaw_Rate_Pid, Yaw_Rate_Desired);

      Rate_Pid.Update (Pitch_Rate_Pid, Pitch_Rate_Actual, True);
      Rate_Pid.Update (Roll_Rate_Pid, Roll_Rate_Actual, True);
      Rate_Pid.Update (Yaw_Rate_Pid, Yaw_Rate_Actual, True);
   end Correct_Rate_PID;

   --------------------------
   -- Correct_Attitude_Pid --
   --------------------------

   procedure Correct_Attitude_Pid
     (Euler_Roll_Actual   : Types.T_Degrees;
      Euler_Pitch_Actual  : Types.T_Degrees;
      Euler_Yaw_Actual    : Types.T_Degrees;
      Euler_Roll_Desired  : Types.T_Degrees;
      Euler_Pitch_Desired : Types.T_Degrees;
      Euler_Yaw_Desired   : Types.T_Degrees)
   is
      Yaw_Error : Float := Euler_Yaw_Desired - Euler_Yaw_Actual;
   begin
      Attitude_Pid.Set_Desired (Roll_Pid, Euler_Roll_Desired);
      Attitude_Pid.Set_Desired (Pitch_Pid, Euler_Pitch_Desired);

      Attitude_Pid.Update (Roll_Pid, Euler_Roll_Actual, True);
      Attitude_Pid.Update (Pitch_Pid, Euler_Pitch_Actual, True);

      --  Special case for Yaw axis
      if Yaw_Error > 180.0 then
         Yaw_Error := Yaw_Error - 360.0;
      elsif Yaw_Error < -180.0 then
         Yaw_Error := Yaw_Error + 360.0;
      end if;

      Attitude_Pid.Set_Error (Yaw_Pid, Yaw_Error);
      Attitude_Pid.Update (Yaw_Pid, Euler_Yaw_Actual, False);
   end Correct_Attitude_Pid;

   -------------------
   -- Reset_All_Pid --
   -------------------

   procedure Reset_All_Pid is
   begin
      Rate_Pid.Reset (Roll_Rate_Pid);
      Attitude_Pid.Reset (Roll_Pid);
      Rate_Pid.Reset (Pitch_Rate_Pid);
      Attitude_Pid.Reset (Pitch_Pid);
      Rate_Pid.Reset (Yaw_Rate_Pid);
      Attitude_Pid.Reset (Yaw_Pid);
   end Reset_All_Pid;

   -------------------------
   -- Get_Actuator_Output --
   -------------------------

   procedure Get_Actuator_Output
     (Actuator_Roll  : out Types.T_Int16;
      Actuator_Pitch : out Types.T_Int16;
      Actuator_Yaw   : out Types.T_Int16) is
   begin
      Actuator_Roll := Safety.Truncate_To_T_Int16
        (Rate_Pid.Get_Output (Roll_Rate_Pid));
      Actuator_Pitch := Safety.Truncate_To_T_Int16
        (Rate_Pid.Get_Output (Pitch_Rate_Pid));
      Actuator_Yaw := Safety.Truncate_To_T_Int16
        (Rate_Pid.Get_Output (Yaw_Rate_Pid));
   end Get_Actuator_Output;

   ----------------------
   -- Get_Desired_Rate --
   ----------------------

   procedure Get_Desired_Rate
     (Roll_Rate_Desired  : out Float;
      Pitch_Rate_Desired : out Float;
      Yaw_Rate_Desired   : out Float) is
   begin
      Roll_Rate_Desired := Attitude_Pid.Get_Output (Roll_Pid);
      Pitch_Rate_Desired := Attitude_Pid.Get_Output (Pitch_Pid);
      Yaw_Rate_Desired := Attitude_Pid.Get_Output (Yaw_Pid);
   end Get_Desired_Rate;

end Controller;
