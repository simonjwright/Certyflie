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
with Types;

package SensFusion6
with SPARK_Mode,
  Abstract_State => (SensFusion6_State),
  Initializes    => (SensFusion6_State)
is

   --  Procedures and functions

   --  Initialize the sensorfusion module.
   procedure Init;

   --  Test if the sensorfusion module is initialized.
   function Test return Boolean;

   --  Update the quaternions by fusing sensor measurements.
   procedure Update_Q
     (Gx : IMU.T_Rate;
      Gy : IMU.T_Rate;
      Gz : IMU.T_Rate;
      Ax : IMU.T_Acc;
      Ay : IMU.T_Acc;
      Az : IMU.T_Acc;
      Mx : IMU.T_Mag;
      My : IMU.T_Mag;
      Mz : IMU.T_Mag;
      Dt : Types.T_Delta_Time);

   --  Get Euler roll, pitch and yaw from the current quaternions.
   --  Must be called after a call to 'Update_Q' to have
   --  the latest angles.
   procedure Get_Euler_RPY
     (Euler_Roll_Actual  : out Types.T_Degrees;
      Euler_Pitch_Actual : out Types.T_Degrees;
      Euler_Yaw_Actual   : out Types.T_Degrees);

   --  Get accleration along Z axis, without gravity.
   function Get_AccZ_Without_Gravity
     (Ax : IMU.T_Acc;
      Ay : IMU.T_Acc;
      Az : IMU.T_Acc) return Float;

private

   --  Global variables and constants

   Is_Init : Boolean := False with Part_Of => SensFusion6_State;

   Q0 : Types.T_Quaternion := 1.0
     with Part_Of => SensFusion6_State;
   Q1 : Types.T_Quaternion := 0.0
     with Part_Of => SensFusion6_State;
   Q2 : Types.T_Quaternion := 0.0
     with Part_Of => SensFusion6_State;
   --  quaternion of sensor frame relative to auxiliary frame
   Q3 : Types.T_Quaternion := 0.0
     with Part_Of => SensFusion6_State;

   --   Implementation of Madgwick's IMU and AHRS algorithms.
   --   See: http:--  www.x-io.co.uk/open-source-ahrs-with-x-imu
   --
   --   Date     Author          Notes
   --   29/09/2011 SOH Madgwick    Initial release
   --   02/10/2011 SOH Madgwick  Optimised for reduced CPU load

   --  Global variables and constants

   MAX_INTEGRAL_ERROR : constant := 100.0;
   MAX_RATE_CHANGE    : constant := 1_000_000.0;

end SensFusion6;
