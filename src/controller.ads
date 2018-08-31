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

with IMU;
with Pid;
with Pid_Parameters;
with Types;
pragma Elaborate_All (Pid);

package Controller
with SPARK_Mode,
  Abstract_State => (Attitude_PIDs, Rate_PIDs, Controller_State)
is
   --  PID Generic package initizalization
   package Attitude_Pid is new Pid
     (Types.T_Degrees'First,
      Types.T_Degrees'Last,
      Float'First / 4.0,
      Float'Last / 4.0,
      Pid_Parameters.MIN_ATTITUDE_COEFF,
      Pid_Parameters.MAX_ATTITUDE_COEFF);

   package Rate_Pid is new Pid
     (IMU.T_Rate'First,
      IMU.T_Rate'Last,
      Float'First / 4.0,
      Float'Last / 4.0,
      Pid_Parameters.MIN_RATE_COEFF,
      Pid_Parameters.MAX_RATE_COEFF);

   --  Procedures and functions

   --  Initialize all the PID's needed for the drone.
   procedure Init
     with
       Global => (Output => (Attitude_PIDs, Rate_PIDs, Controller_State));

   --  Test if the PID's have been initialized.
   function Test return Boolean
     with
       Global => (Input => Controller_State);

   --  Update the rate PID's for each axis (Roll, Pitch, Yaw)
   --  given the measured values along each axis and the desired
   --  values retrieved from the corresponding
   --  attitude PID's.
   procedure Correct_Rate_PID
     (Roll_Rate_Actual   : IMU.T_Rate;
      Pitch_Rate_Actual  : IMU.T_Rate;
      Yaw_Rate_Actual    : IMU.T_Rate;
      Roll_Rate_Desired  : IMU.T_Rate;
      Pitch_Rate_Desired : IMU.T_Rate;
      Yaw_Rate_Desired   : IMU.T_Rate)
     with
       Global => (In_Out => Rate_PIDs);

   --  Update the attitude PID's for each axis given (Roll, Pitch, Yaw)
   --  given the measured values along each axis and the
   --  desired values retrieved from the commander.
   procedure Correct_Attitude_Pid
     (Euler_Roll_Actual   : Types.T_Degrees;
      Euler_Pitch_Actual  : Types.T_Degrees;
      Euler_Yaw_Actual    : Types.T_Degrees;
      Euler_Roll_Desired  : Types.T_Degrees;
      Euler_Pitch_Desired : Types.T_Degrees;
      Euler_Yaw_Desired   : Types.T_Degrees)
     with
       Global => (In_Out => Attitude_PIDs);

   --  Reset all the PID's error values.
   procedure Reset_All_Pid
     with
       Global => (In_Out => (Attitude_PIDs, Rate_PIDs));

   --  Get the output of the rate PID's.
   --  Must be called after 'Correct_Rate_Pid' to update the PID's.
   procedure Get_Actuator_Output
     (Actuator_Roll  : out Types.T_Int16;
      Actuator_Pitch : out Types.T_Int16;
      Actuator_Yaw   : out Types.T_Int16)
     with
       Global => (Input => Rate_PIDs);

   --  Get the output of the attitude PID's, which will command the rate PID's.
   --  Must be called after 'Correct_Attitude_Pid' to update
   --  the PID's.
   procedure Get_Desired_Rate
     (Roll_Rate_Desired  : out Float;
      Pitch_Rate_Desired : out Float;
      Yaw_Rate_Desired   : out Float)
     with
       Global => (Input => Attitude_PIDs);

private

   --  Global variables

   Roll_Pid       : Attitude_Pid.Object with Part_Of => Attitude_PIDs;
   Pitch_Pid      : Attitude_Pid.Object with Part_Of => Attitude_PIDs;
   Yaw_Pid        : Attitude_Pid.Object with Part_Of => Attitude_PIDs;

   Roll_Rate_Pid  : Rate_Pid.Object with Part_Of => Rate_PIDs;
   Pitch_Rate_Pid : Rate_Pid.Object with Part_Of => Rate_PIDs;
   Yaw_Rate_Pid   : Rate_Pid.Object with Part_Of => Rate_PIDs;

   Is_Init : Boolean := False with Part_Of => Controller_State;

end Controller;
