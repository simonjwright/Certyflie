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

package body Pid is

   ----------
   -- Init --
   ----------

   procedure Init
     (Pid          : out Object;
      Desired      : T_Input;
      Kp           : T_Coeff;
      Ki           : T_Coeff;
      Kd           : T_Coeff;
      I_Limit_Low  : T_I_Limit;
      I_Limit_High : T_I_Limit;
      Dt           : Types.T_Delta_Time) is
   begin
      Pid.Desired := Desired;
      Pid.Error := 0.0;
      Pid.Prev_Error := 0.0;
      Pid.Integ := 0.0;
      Pid.Deriv := 0.0;
      Pid.Kp := Kp;
      Pid.Ki := Ki;
      Pid.Kd := Kd;
      Pid.Out_P := 0.0;
      Pid.Out_I := 0.0;
      Pid.Out_D := 0.0;
      Pid.I_Limit_Low  := I_Limit_Low;
      Pid.I_Limit_High := I_Limit_High;
      Pid.Dt := Dt;
   end Init;

   -----------
   -- Reset --
   -----------

   procedure Reset (Pid : in out Object) is
   begin
      Pid.Error := 0.0;
      Pid.Prev_Error := 0.0;
      Pid.Integ := 0.0;
      Pid.Deriv := 0.0;
   end Reset;

   ------------
   -- Update --
   ------------

   procedure Update
     (Pid          : in out Object;
      Measured     : T_Input;
      Update_Error : Boolean) is
   begin
      if Update_Error then
         Pid.Error := Pid.Desired - Measured;
      end if;

      pragma Assert (Pid.Error * Pid.Dt in
                       T_Error'First * 2.0 * Types.T_Delta_Time'Last ..
                         T_Error'Last * 2.0 * Types.T_Delta_Time'Last);

      Pid.Integ := Safety.Saturate (Pid.Integ + Pid.Error * Pid.Dt,
                                    Pid.I_Limit_Low,
                                    Pid.I_Limit_High);

      Pid.Deriv := (Pid.Error - Pid.Prev_Error) / Pid.Dt;

      Pid.Out_P := Pid.Kp * Pid.Error;

      pragma Assert (Pid.Integ in T_I_Limit'First * 1.0 ..
                       T_I_Limit'Last * 1.0);
      Pid.Out_I := Pid.Ki * Pid.Integ;

      Pid.Out_D := Pid.Kd * Pid.Deriv;

      Pid.Prev_Error := Pid.Error;
   end Update;

   ----------------
   -- Get_Output --
   ----------------

   function Get_Output (Pid : Object) return Float is
     (Pid.Out_P + Pid.Out_I + Pid.Out_D);

   ---------------
   -- Is_Active --
   ---------------

   function Is_Active (Pid : Object) return Boolean
   is
      Is_Active : Boolean := True;
   begin
      if Pid.Kp < 0.0001 and Pid.Ki < 0.0001 and Pid.Kd < 0.0001 then
         Is_Active := False;
      end if;

      return Is_Active;
   end Is_Active;

   -----------------
   -- Set_Desired --
   -----------------

   procedure Set_Desired
     (Pid     : in out Object;
      Desired : T_Input) is
   begin
      Pid.Desired := Desired;
   end Set_Desired;

   -----------------
   -- Get_Desired --
   -----------------

   function Get_Desired (Pid : Object) return Float is
     (Pid.Desired);

   ---------------
   -- Set_Error --
   ---------------

   procedure Set_Error
     (Pid   : in out Object;
      Error : T_Error) is
   begin
      Pid.Error := Error;
   end Set_Error;

   ------------
   -- Set_Kp --
   ------------

   procedure Set_Kp
     (Pid : in out Object;
      Kp  : T_Coeff) is
   begin
      Pid.Kp := Kp;
   end Set_Kp;

   ------------
   -- Set_Ki --
   ------------

   procedure Set_Ki
     (Pid : in out Object;
      Ki  : T_Coeff) is
   begin
      Pid.Ki := Ki;
   end Set_Ki;

   ------------
   -- Set_Kd --
   ------------

   procedure Set_Kd
     (Pid : in out Object;
      Kd  : T_Coeff) is
   begin
      Pid.Kd := Kd;
   end Set_Kd;

   ---------------------
   -- Set_I_Limit_Low --
   ---------------------

   procedure Set_I_Limit_Low
     (Pid          : in out Object;
      I_Limit_Low  : T_I_Limit) is
   begin
      Pid.I_Limit_Low := I_Limit_Low;
   end Set_I_Limit_Low;

   ----------------------
   -- Set_I_Limit_High --
   ----------------------

   procedure Set_I_Limit_High
     (Pid            : in out Object;
      I_Limit_High   : T_I_Limit) is
   begin
      Pid.I_Limit_High := I_Limit_High;
   end Set_I_Limit_High;

   ------------
   -- Set_Dt --
   ------------

   procedure Set_Dt
     (Pid : in out Object;
      Dt  : Types.T_Delta_Time) is
   begin
      Pid.Dt := Dt;
   end Set_Dt;

   ---------------
   -- Get_Integ --
   ---------------

   function Get_Integral_Term (Pid : Object) return T_I_Limit
   is (Pid.Integ);

   ---------------
   -- Set_Integ --
   ---------------

   procedure Set_Integral_Term (Pid : in out Object; Integ : T_I_Limit) is
   begin
      Pid.Integ := Integ;
   end Set_Integral_Term;

end Pid;
