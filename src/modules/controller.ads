with Types; use Types;
with IMU; use IMU;
with Pid_Parameters; use Pid_Parameters;
with Pid;
pragma Elaborate_All (Pid);

package Controller
with SPARK_Mode,
  Abstract_State => (Attitude_PIDs, Rate_PIDs, Controller_State)
is
   --  PID Generic package initizalization
   package Attitude_Pid is new Pid
     (T_Degrees'First,
      T_Degrees'Last,
      Float'First / 4.0,
      Float'Last / 4.0,
      MIN_ATTITUDE_COEFF,
      MAX_ATTITUDE_COEFF);

   package Rate_Pid is new Pid
     (T_Rate'First,
      T_Rate'Last,
      Float'First / 4.0,
      Float'Last / 4.0,
      MIN_RATE_COEFF,
      MAX_RATE_COEFF);

   --  Procedures and functions

   --  Initalize all the PID's needed for the drone.
   procedure Controller_Init
     with
       Global => (Output => (Attitude_PIDs, Rate_PIDs, Controller_State));

   --  Test if the PID's have been initialized.
   function Controller_Test return Boolean
     with
       Global => (Input => Controller_State);

   --  Update the rate PID's for each axis (Roll, Pitch, Yaw)
   --  given the measured values along each axis and the desired
   --  values retrieved from the corresponding
   --  attitude PID's.
   procedure Controller_Correct_Rate_PID
     (Roll_Rate_Actual   : T_Rate;
      Pitch_Rate_Actual  : T_Rate;
      Yaw_Rate_Actual    : T_Rate;
      Roll_Rate_Desired  : T_Rate;
      Pitch_Rate_Desired : T_Rate;
      Yaw_Rate_Desired   : T_Rate)
     with
       Global => (In_Out => Rate_PIDs);

   --  Update the attitude PID's for each axis given (Roll, Pitch, Yaw)
   --  given the measured values along each axis and the
   --  desired values retrieved from the commander.
   procedure Controller_Correct_Attitude_Pid
     (Euler_Roll_Actual   : T_Degrees;
      Euler_Pitch_Actual  : T_Degrees;
      Euler_Yaw_Actual    : T_Degrees;
      Euler_Roll_Desired  : T_Degrees;
      Euler_Pitch_Desired : T_Degrees;
      Euler_Yaw_Desired   : T_Degrees)
     with
       Global => (In_Out => Attitude_PIDs);

   --  Reset all the PID's error values.
   procedure Controller_Reset_All_Pid
     with
       Global => (In_Out => (Attitude_PIDs, Rate_PIDs));

   --  Get the output of the rate PID's.
   --  Must be called after 'Controller_Correct_Rate_Pid' to update the PID's.
   procedure Controller_Get_Actuator_Output
     (Actuator_Roll  : out T_Int16;
      Actuator_Pitch : out T_Int16;
      Actuator_Yaw   : out T_Int16)
     with
       Global => (Input => Rate_PIDs);

   --  Get the output of the attitude PID's, which will command the rate PID's.
   --  Must be called after 'Controller_Correct_Attitude_Pid' to update
   --  the PID's.
   procedure Controller_Get_Desired_Rate
     (Roll_Rate_Desired  : out Float;
      Pitch_Rate_Desired : out Float;
      Yaw_Rate_Desired   : out Float)
     with
       Global => (Input => Attitude_PIDs);

private

   --  Global variables

   Roll_Pid       : Attitude_Pid.Pid_Object with Part_Of => Attitude_PIDs;
   Pitch_Pid      : Attitude_Pid.Pid_Object with Part_Of => Attitude_PIDs;
   Yaw_Pid        : Attitude_Pid.Pid_Object with Part_Of => Attitude_PIDs;

   Roll_Rate_Pid  : Rate_Pid.Pid_Object with Part_Of => Rate_PIDs;
   Pitch_Rate_Pid : Rate_Pid.Pid_Object with Part_Of => Rate_PIDs;
   Yaw_Rate_Pid   : Rate_Pid.Pid_Object with Part_Of => Rate_PIDs;

   Is_Init : Boolean := False with Part_Of => Controller_State;

end Controller;