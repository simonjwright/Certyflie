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

with Types;
with Pid_Parameters;

generic
   INPUT_LOW_LIMIT   : Float;
   INPUT_HIGH_LIMIT  : Float;
   OUTPUT_LOW_LIMIT  : Float;
   OUTPUT_HIGH_LIMIT : Float;
   COEFF_LOW_LIMIT   : Float;
   COEFF_HIGH_LIMIT  : Float;

package Pid is

   --  Subtypes for inputs/outputs of the PID.
   subtype T_Input  is Float range INPUT_LOW_LIMIT .. INPUT_HIGH_LIMIT;
   subtype T_Output is Float range OUTPUT_LOW_LIMIT .. OUTPUT_HIGH_LIMIT;
   subtype T_Error  is Float range
     2.0 * INPUT_LOW_LIMIT .. 2.0 * INPUT_HIGH_LIMIT;
   subtype T_Deriv  is Float range
     4.0 * INPUT_LOW_LIMIT / Types.T_Delta_Time'First ..
     4.0 * INPUT_HIGH_LIMIT / Types.T_Delta_Time'First;
   subtype T_I_Limit is Float range
     -Pid_Parameters.DEFAULT_INTEGRATION_LIMIT ..
     Pid_Parameters.DEFAULT_INTEGRATION_LIMIT;
   subtype T_Coeff is Float range COEFF_LOW_LIMIT .. COEFF_HIGH_LIMIT;

   --  Types
   type Object is private;

   --  Procedures and Functions

   --  PID object initialization.
   procedure Init
     (Pid           : out Object;
      Desired       : T_Input;
      Kp            : T_Coeff;
      Ki            : T_Coeff;
      Kd            : T_Coeff;
      I_Limit_Low   : T_I_Limit;
      I_Limit_High  : T_I_Limit;
      Dt            : Types.T_Delta_Time);

   --  Reset the PID error values.
   procedure Reset (Pid : in out Object);

   --  Update the PID parameters. Set 'UpdateError' to 'False' if error
   --  has been set previously for a special calculation with 'PidSetError'.
   procedure Update
     (Pid          : in out Object;
      Measured     : T_Input;
      Update_Error : Boolean);

   --  Return the PID output. Must be called after 'Update'.
   function Get_Output (Pid : Object) return Float;

   --  Find out if the PID is active.
   function Is_Active (Pid : Object) return Boolean;

   --  Set a new set point for the PID to track.
   procedure Set_Desired
     (Pid     : in out Object;
      Desired : T_Input);

   --  Get the PID desired set point.
   function Get_Desired (Pid : Object) return Float;

   --  Set the new error. Used if special calculation is needed.
   procedure Set_Error
     (Pid   : in out Object;
      Error : T_Error);

   --  Set a new proprtional gain for the PID.
   procedure Set_Kp
     (Pid : in out Object;
      Kp  : T_Coeff);

   --  Set a new integral gain for the PID.
   procedure Set_Ki
     (Pid : in out Object;
      Ki  : T_Coeff);

   --  Set a new derivative gain for the PID.
   procedure Set_Kd
     (Pid : in out Object;
      Kd  : T_Coeff);

   --  Set a new low limit for the integral term.
   procedure Set_I_Limit_Low
     (Pid          : in out Object;
      I_Limit_Low  : T_I_Limit);

   --  Set a new high limit for the integral term.
   procedure Set_I_Limit_High
     (Pid           : in out Object;
      I_Limit_High  : T_I_Limit);

   --  Set a new dt gain for the PID.
   --  Defaults to IMU_UPDATE_DT upon construction. ???
   procedure Set_Dt
     (Pid : in out Object;
      Dt  : Types.T_Delta_Time);

   --  Used for Altitude Hold.
   function Get_Integral_Term (Pid : Object) return T_I_Limit;
   procedure Set_Integral_Term (Pid : in out Object; Integ : T_I_Limit);

private

   type Object is record
      Desired      : T_Input            := 0.0; --  Set point
      Error        : T_Error            := 0.0; --  Error
      Prev_Error   : T_Error            := 0.0; --  Previous Error
      Integ        : T_I_Limit          := 0.0; --  Integral
      Deriv        : T_Deriv            := 0.0; --  Derivative
      Kp           : T_Coeff            := 0.0; --  Proportional Gain
      Ki           : T_Coeff            := 0.0; --  Integral Gain
      Kd           : T_Coeff            := 0.0; --  Derivative Gain
      Out_P        : T_Output           := 0.0; --  Proportional Output (debug)
      Out_I        : T_Output           := 0.0; --  Integral Output (debug)
      Out_D        : T_Output           := 0.0; --  Derivative Output (debug)
      I_Limit_Low  : T_I_Limit          := 0.0; --  Limit of integral term
      I_Limit_High : T_I_Limit          := 0.0; --  Limit of integral term
      Dt           : Types.T_Delta_Time := 0.1; --  Delta Time
   end record;
   pragma Convention (C, Object);

end Pid;
