with Ada.Real_Time;
with Stabilizer_Types;

private with Ada.Numerics.Real_Arrays;

package Kalman_Core is

   type Core is limited private;

   procedure Initialize (This    : in out Core;
                         At_Time :        Ada.Real_Time.Time);

   procedure Predict
     (This      : in out Core;
      Thrust    :        Float;  -- XXX units?
      Acc       :        Stabilizer_Types.Acceleration;  -- MKS
      Gyro      :        Stabilizer_Types.Angular_Velocity_Radians;  -- MKS
      Interval  :        Ada.Real_Time.Time_Span;
      Is_Flying :        Boolean);

   procedure Add_Process_Noise (This     : in out Core;
                                At_Time  :        Ada.Real_Time.Time);

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

   type Component_Names is (C_X, C_Y, C_Z, C_Vx, C_Vy, C_Vz, C_D0, C_D1, C_D2);
   X  : constant := Component_Names'Pos (C_X);
   Y  : constant := Component_Names'Pos (C_Y);
   Z  : constant := Component_Names'Pos (C_Z);
   Vx : constant := Component_Names'Pos (C_Vx);
   Vy : constant := Component_Names'Pos (C_Vy);
   Vz : constant := Component_Names'Pos (C_Vz);
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

   type Core is limited record
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
   --  The C code regards Measurement_Vector as being a column vector.
   procedure Scalar_Update (This               : in out Core;
                            Measurement_Vector :        Space_Vector;
                            Error              :        Float;
                            Standard_Deviation :        Float);

end Kalman_Core;
