------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--           Copyright (C) 2021, Simon Wright <simon@pushface.org>          --
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

with Ada.Real_Time;
with Stabilizer_Types;

private with Ada.Numerics.Real_Arrays;

package Kalman_Core is

   type Core is private;

   procedure Initialize (This    : in out Core;
                         At_Time :        Ada.Real_Time.Time);

   procedure Predict
     (This      : in out Core;
      Thrust    :        Float;  -- XXX units?
      Acc       :        Stabilizer_Types.Acceleration;  -- MKS
      Gyro      :        Stabilizer_Types.Angular_Velocity_Radians;  -- MKS
      Interval  :        Ada.Real_Time.Time_Span;
      Is_Flying :        Boolean);

   procedure Add_Process_Noise (This    : in out Core;
                                At_Time :        Ada.Real_Time.Time);

   procedure Update_With_Baro (This      : in out Core;
                               Altitude  :        Float);
   --  Altitude in metres above sea level.

   procedure Update_With_Flow
     (This : in out Core;
      Flow :        Stabilizer_Types.Flow_Measurement;
      Gyro :        Stabilizer_Types.Angular_Velocity_Degrees);

   procedure Update_With_ToF
     (This : in out Core;
      ToF  :        Stabilizer_Types.ToF_Measurement);

   function Has_Been_Updated (This : Core) return Boolean;

   procedure Finalize (This : in out Core);
   --  XXX should we just check This.Updated internally?

   procedure Externalize_State
     (This    :     Core;
      State   : out Stabilizer_Types.State_Data;
      Acc     :     Stabilizer_Types.Acceleration;  -- G
      At_Time :     Ada.Real_Time.Time);

private

   subtype Space_Vector is Ada.Numerics.Real_Arrays.Real_Vector (0 .. 8);

   --  Safe way of naming the components of Space_Vector.

   type Component_Names is (C_X, C_Y, C_Z, C_Px, C_Py, C_Pz, C_D0, C_D1, C_D2);
   X  : constant := Component_Names'Pos (C_X);
   Y  : constant := Component_Names'Pos (C_Y);
   Z  : constant := Component_Names'Pos (C_Z);
   Px : constant := Component_Names'Pos (C_Px);
   Py : constant := Component_Names'Pos (C_Py);
   Pz : constant := Component_Names'Pos (C_Pz);
   D0 : constant := Component_Names'Pos (C_D0);
   D1 : constant := Component_Names'Pos (C_D1);
   D2 : constant := Component_Names'Pos (C_D2);

   subtype Space_Matrix is Ada.Numerics.Real_Arrays.Real_Matrix
     (Space_Vector'Range, Space_Vector'Range);

   type Quaternion is record
      W : Float;
      X : Float;
      Y : Float;
      Z : Float;
   end record;

   type Core is record
      --  State vector
      S : Space_Vector := (others => 0.0);
      --  Attitude as quaternion
      Q : Quaternion;
      --  Attitude as matrix
      R : Ada.Numerics.Real_Arrays.Real_Matrix (0 .. 2, 0 .. 2)
        := (others => (others => 0.0));
      --  Covariance
      P : Space_Matrix := (others => (others => 0.0));

      Barometer_Reference_Height     : Float;
      Quad_Is_Flying                 : Boolean := False;
      Updated                        : Boolean := False;
      Reset_Estimation               : Boolean := True;
      Last_Prediction_Time           : Ada.Real_Time.Time;
      Last_Process_Noise_Update_Time : Ada.Real_Time.Time;
      Last_Baro_Update_Time          : Ada.Real_Time.Time;
      Last_Flight_Command_Time       : Ada.Real_Time.Time;
      Takeoff_Time                   : Ada.Real_Time.Time;
   end record;

   function Has_Been_Updated (This : Core) return Boolean
   is (This.Updated);

   function Has_Nan (This : Core) return Boolean;

   function State_Is_Within_Bounds (This : Core) return Boolean;

   --  This updates one of the elements of the state vector (the
   --  corresponding element is indicated by 1.0 in that element of
   --  Measurement_Vector, the others being 0.0).
   --
   --  The C code regards Measurement_Vector as being a column vector,
   --  but this is only so as to get the correct matrix operation; we
   --  get the same effect by multiplication order.
   procedure Scalar_Update (This               : in out Core;
                            Measurement_Vector :        Space_Vector;
                            Error              :        Float;
                            Standard_Deviation :        Float);

end Kalman_Core;
