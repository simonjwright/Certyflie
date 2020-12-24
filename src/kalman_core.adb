with Ada.Numerics.Elementary_Functions;

with Console;

package body Kalman_Core is

   use Ada.Numerics.Elementary_Functions;
   use Ada.Numerics.Real_Arrays;

   G : constant := 9.81;  -- m/s/s

   --  Initial yaw of the Crazyflie in radians.
   --  0 --- facing positive X
   --  PI / 2 --- facing positive Y
   --  PI --- facing negative X
   --  3 * PI / 2 --- facing negative Y
   Initial_Yaw : constant Float := 0.0;
   Initial_Quaternion : constant Quaternion
     := (Cos (Initial_Yaw / 2.0),
         0.0,
         0.0,
         Sin (Initial_Yaw / 2.0));

   --  Initial standard deviations, uncertain of position, but know
   --  we're stationary and roughly flat
   Initial_SD_Position_XY : constant := 100.0;
   Initial_SD_Position_Z  : constant := 1.0;
   Initial_SD_Velocity    : constant := 0.01;
   Initial_SD_Roll_Pitch  : constant := 0.01;
   Initial_SD_Yaw         : constant := 0.01;

   --  In estimator_kalman.c
   Max_Covariance : constant := 100.0;
   Min_Covariance : constant := 1.0e-6;

   type Process_Noise_T is record
      Acc_X_Y         : Float;
      Acc_Z           : Float;
      Vel             : Float;
      Pos             : Float;
      Baro            : Float; -- metres???
      Att             : Float; -- radians
      Gyro_Roll_Pitch : Float; -- radians/sec
      Gyro_Yaw        : Float; -- radians/sec
   end record;
   Process_Noise : constant Process_Noise_T
     := (Acc_X_Y         => 0.5,
         Acc_Z           => 1.0,
         Vel             => 1.0,  -- was 0.0
         Pos             => 5.0,  -- was 0.0
         Baro            => 2.0,
         Att             => 0.0,
         Gyro_Roll_Pitch => 0.1,
         Gyro_Yaw        => 0.1);

   procedure Initialize (This    : in out Core;
                         At_Time :        Ada.Real_Time.Time)
   is
   begin
      Console.Put_Line ("Resetting Kalman state");

      This.S := (others => 0.0);
      This.R := Unit_Matrix
        (This.R'Length (1), This.R'First (1), This.R'First (2));
      This.Q := Initial_Quaternion;

      This.P := (others => (others => 0.0));

      This.P (X, X) := Initial_SD_Position_XY**2;
      This.P (Y, Y) := Initial_SD_Position_XY**2;
      This.P (Z, Z) := Initial_SD_Position_XY**2;

      This.P (Vx, Vx) := Initial_SD_Velocity**2;
      This.P (Vy, Vy) := Initial_SD_Velocity**2;
      This.P (Vz, Vz) := Initial_SD_Velocity**2;

      This.P (D0, D0) := Initial_SD_Roll_Pitch**2;
      This.P (D1, D1) := Initial_SD_Roll_Pitch**2;
      This.P (D2, D2) := Initial_SD_Yaw**2;

      This.Barometer_Reference_Height := 0.0;

      This.Last_Prediction_Time := At_Time;
      This.Last_Process_Noise_Update_Time := At_Time;
      This.Last_Baro_Update_Time := At_Time;
      This.Last_Flight_Command_Time := At_Time;  -- XXX

      This.Quad_Is_Flying := False;
      This.Updated := False;
      This.Reset_Estimation := False;
   end Initialize;

   function Is_Nan (V : Float) return Boolean
   is (V /= 0.0 and then (not (V < 0.0) and then not (V > 0.0)));

   function Has_Nan (This : Core) return Boolean
   is ((for some S of This.S => Is_Nan (S))
         or else (for some P of This.P => Is_Nan (P))
         or else (for some R of This.R => Is_Nan (R))
         or else Is_Nan (This.Q.W)
         or else Is_Nan (This.Q.X)
         or else Is_Nan (This.Q.Y)
         or else Is_Nan (This.Q.Z));

   procedure Predict
     (This      : in out Core;
      Thrust    :        Float;
      Acc       :        Stabilizer_Types.Acceleration;
      Gyro      :        Stabilizer_Types.Angular_Velocity_Radians;
      Interval  :        Ada.Real_Time.Time_Span;
      Is_Flying :        Boolean)
   is
      --  See C modules/kalman_core.c, just after if (quadIsFlying) test
      pragma Unreferenced (Thrust);

      Dt : constant Float := Float (Ada.Real_Time.To_Duration (Interval));
      --  Matrix to rotate attitude covariances
      A : Space_Matrix
        := Unit_Matrix (Space_Matrix'Length (1),
                        Space_Matrix'First (1),
                        Space_Matrix'First (2));
      Old_P : constant Space_Matrix := This.P;
   begin
      pragma Assert (not Has_Nan (This));

      This.Updated := True;

      --  Position from body-frame velocity
      A (X, Vx) := This.R (0, 0) * Dt;
      A (Y, Vx) := This.R (1, 0) * Dt;
      A (Z, Vx) := This.R (2, 0) * Dt;

      A (X, Vy) := This.R (0, 1) * Dt;
      A (Y, Vy) := This.R (1, 1) * Dt;
      A (Z, Vy) := This.R (2, 1) * Dt;

      A (X, Vz) := This.R (0, 2) * Dt;
      A (Y, Vz) := This.R (1, 2) * Dt;
      A (Z, Vz) := This.R (2, 2) * Dt;

      --  Position from attitude error
      A (X, D0) :=
        (This.S (Vy) * This.R (0, 2) - This.S (Vz) * This.R (0, 1))
           * Dt;
      A (Y, D0) :=
        (This.S (Vy) * This.R (1, 2) - This.S (Vz) * This.R (1, 1))
           * Dt;
      A (Z, D0) :=
        (This.S (Vy) * This.R (2, 2) - This.S (Vz) * This.R (2, 1))
           * Dt;

      A (X, D1) :=
        (-This.S (Vx) * This.R (0, 2) + This.S (Vz) * This.R (0, 0))
           * Dt;
      A (Y, D1) :=
        (-This.S (Vx) * This.R (1, 2) + This.S (Vz) * This.R (1, 0))
           * Dt;
      A (Z, D1) :=
        (-This.S (Vx) * This.R (2, 2) + This.S (Vz) * This.R (2, 0))
           * Dt;

      A (X, D2) :=
        (This.S (Vx) * This.R (0, 1) - This.S (Vy) * This.R (0, 0))
           * Dt;
      A (Y, D2) :=
        (This.S (Vx) * This.R (1, 1) - This.S (Vy) * This.R (1, 0))
           * Dt;
      A (Z, D2) :=
        (This.S (Vx) * This.R (2, 1) - This.S (Vy) * This.R (2, 0))
           * Dt;

      --  Body-frame velocity from body-frame velocity
      A (Vx, Vx) := 1.0;  -- drag negligible
      A (Vy, Vx) := -Gyro.Z * Dt;
      A (Vz, Vx) := Gyro.Y * Dt;

      A (Vx, Vy) := Gyro.Z * Dt;
      A (Vy, Vy) := 1.0;
      A (Vz, Vy) := -Gyro.X * Dt;

      A (Vx, Vz) := -Gyro.Y * Dt;
      A (Vy, Vz) := Gyro.X * Dt;
      A (Vz, Vz) := 1.0;

      --  Body-frame velocity from attitude error
      A (Vx, D0) := 0.0;
      A (Vy, D0) := -G * This.R (2, 2) * Dt;
      A (Vz, D0) := G * This.R (2, 1) * Dt;

      A (Vx, D1) := G * This.R (2, 2) * Dt;
      A (Vy, D1) := 0.0;
      A (Vz, D1) := -G * This.R (2, 0) * Dt;

      A (Vx, D2) := -G * This.R (2, 1) * Dt;
      A (Vy, D2) := G * This.R (2, 0) * Dt;
      A (Vz, D2) := 0.0;

      --  Attitude error from attitude error
      declare
         G0 : constant Float := Gyro.X * Dt / 2.0;
         G1 : constant Float := Gyro.Y * Dt / 2.0;
         G2 : constant Float := Gyro.Z * Dt / 2.0;
      begin
         A (D0, D0) := 1.0 - G1**2 / 2.0 - G2**2 / 2.0;
         A (D0, D1) := G2 + G0 * G1 / 2.0;
         A (D0, D2) := -G1 + G0 * G2 / 2.0;

         A (D1, D0) := -G2 + G0 * G1 / 2.0;
         A (D1, D1) := 1.0 - G0**2 / 2.0 - G2**2 / 2.0;
         A (D1, D2) := G0 + G1 * G2 / 2.0;

         A (D2, D0) := G1 + G0 * G2 / 2.0;
         A (D2, D1) :=  -G0 + G1 * G2 / 2.0;
         A (D2, D2) := 1.0 - G0**2 / 2.0 - G1**2 / 2.0;
      end;

      --  declare
      --     Variation : constant Float := abs (Determinant (A) - 1.0);
      --  begin
      --     pragma Assert (Variation < 1.0e-5,
      --                    "variation is " & Variation'Image);
      --     null;
      --  end;

      --  Covariance update
      This.P := A * This.P * Transpose (A);
      pragma Assert (not Has_Nan (This));

      --  Prediction step
      if Is_Flying then
         --  Acceleration must be in Z (body) direction
         declare
            --  Acceleration could be thrust / mass
            Z_Acc : constant Float := Acc.Z;

            --  Position updates in the body frame (will be rotated to
            --  inertial frame)
            Dx : constant Float := This.S (Vx) * Dt;
            Dy : constant Float := This.S (Vy) * Dt;
            Dz : constant Float := This.S (Vz) * Dt + Z_Acc * Dt**2 / 2.0;

            --  Remember the old velocity for the update
            Old_Vx : constant Float := This.S (Vx);
            Old_Vy : constant Float := This.S (Vy);
            Old_Vz : constant Float := This.S (Vz);
         begin
            --  Position update (XXX in the inertial frame?)
            This.S (X) := This.S (X)
              + This.R (0, 0) * Dx
              + This.R (0, 1) * Dy
              + This.R (0, 2) * Dz;
            This.S (Y) := This.S (Y)
              + This.R (1, 0) * Dx
              + This.R (1, 1) * Dy
              + This.R (1, 2) * Dz;
            This.S (Z) := This.S (Z)
              + This.R (2, 0) * Dx
              + This.R (2, 1) * Dy
              + This.R (2, 2) * Dz - G * Dt**2 / 2.0;

            --  Body-velocity update: accelerometers - gyros cross
            --  velocity - gravity in body frame
            This.S (Vx) := This.S (Vx)
              + Dt * (Gyro.Z * Old_Vy
                        - Gyro.Y * Old_Vz
                        - G * This.R (2, 0));
            This.S (Vy) := This.S (Vy)
              + Dt * (-Gyro.Z * Old_Vx
                        + Gyro.X * Old_Vz
                        - G * This.R (2, 1));
            This.S (Vz) := This.S (Vz)
              + Dt * (Z_Acc
                        + Gyro.Y * Old_Vx
                        - Gyro.X * Old_Vy
                        - G * This.R (2, 2));
         end;
      else
         --  Not flying
         --
         --  Acceleration can be in any direction, as measured by
         --  the accelerometer. This occurs, eg. in freefall or
         --  while being carried.
         declare
            Dx : constant Float := This.S (Vx) * Dt + Acc.X * Dt**2 / 2.0;
            Dy : constant Float := This.S (Vy) * Dt + Acc.Y * Dt**2 / 2.0;
            Dz : constant Float := This.S (Vz) * Dt + Acc.Z * Dt**2 / 2.0;
            --  Remember the old velocity for the update
            Old_Vx : constant Float := This.S (Vx);
            Old_Vy : constant Float := This.S (Vy);
            Old_Vz : constant Float := This.S (Vz);
         begin
            --  Position update
            This.S (X) := This.S (X)
              + This.R (0, 0) * Dx
              + This.R (0, 1) * Dy
              + This.R (0, 2) * Dz;
            This.S (Y) := This.S (Y)
              + This.R (1, 0) * Dx
              + This.R (1, 1) * Dy
              + This.R (1, 2) * Dz;
            This.S (Z) := This.S (Z)
              + This.R (2, 0) * Dx
              + This.R (2, 1) * Dy
              + This.R (2, 2) * Dz - G * Dt**2 / 2.0;

            --  Body-velocity update: accelerometers - gyros cross
            --  velocity - gravity in body frame
            This.S (Vx) := This.S (Vx)
              + Dt * (Acc.X
                        + Gyro.Z * Old_Vy
                        - Gyro.Y * Old_Vz
                        - G * This.R (2, 0));
            This.S (Vy) := This.S (Vy)
              + Dt * (Acc.Y
                        - Gyro.Z * Old_Vx
                        + Gyro.X * Old_Vz
                        - G * This.R (2, 1));
            This.S (Vz) := This.S (Vz)
              + Dt * (Acc.Z
                        + Gyro.Y * Old_Vx
                        - Gyro.X * Old_Vy
                        - G * This.R (2, 2));
         end;
      end if;
      pragma Assert (not Has_Nan (This));

      --  CHECKED TO HERE (AGAIN)

      --  Attitude Update (rotate by gyroscope), we do this in
      --  quaternions.  This is the gyroscope angular velocity
      --  integrated over the sample period
      declare
         Dtwx : constant Float := Dt * Gyro.X;
         Dtwy : constant Float := Dt * Gyro.Y;
         Dtwz : constant Float := Dt * Gyro.Z;
         Angle : constant Float
           := Sqrt (Dtwx**2 + Dtwy**2 + Dtwz**2);
         Ca : constant Float := Cos (Angle / 2.0);
         Sa : constant Float := Sin (Angle / 2.0);
         Dq : constant Quaternion
           := (if Angle /= 0.0  -- need a better check, really
               then (Ca,
                     Sa * Dtwx / Angle,
                     Sa * Dtwy / Angle,
                     Sa * Dtwz / Angle)
               else (1.0, 0.0, 0.0, 0.0));
         Tq : Quaternion
           := (Dq.W * This.Q.W - Dq.X * This.Q.X
                 - Dq.Y * This.Q.Y - Dq.Z * This.Q.Z,
               Dq.X * This.Q.W + Dq.W * This.Q.X
                 + Dq.Z * This.Q.Y - Dq.Y * This.Q.Z,
               Dq.Y * This.Q.W - Dq.Z * This.Q.X
                 + Dq.W * This.Q.Y + Dq.X * This.Q.Z,
               Dq.Z * This.Q.W + Dq.Y * This.Q.X
                 - Dq.X * This.Q.Y + Dq.W * This.Q.Z);
         ROLL_PITCH_REVERSION : constant := 0.001;
         --  Value, in kalman_core.c, if LPS_2D_POSITION_HEIGHT
         --  isn't defined
         R : constant := ROLL_PITCH_REVERSION;
         K : constant := 1.0 - R;
      begin
         if not Is_Flying then
            Tq.W := K * Tq.W + R * Initial_Quaternion.W;
            Tq.X := K * Tq.X + R * Initial_Quaternion.X;
            Tq.Y := K * Tq.Y + R * Initial_Quaternion.Y;
            Tq.Z := K * Tq.Z + R * Initial_Quaternion.Z;
         end if;

         --  Normalize and store the result
         declare
            Norm : constant Float := Sqrt (Tq.W * Tq.W
                                             + Tq.X * Tq.X
                                             + Tq.Y * Tq.Y
                                             + Tq.Z * Tq.Z);
         begin
            This.Q.W := Tq.W / Norm;
            This.Q.X := Tq.X / Norm;
            This.Q.Y := Tq.Y / Norm;
            This.Q.Z := Tq.Z / Norm;
         end;
      end;
      pragma Assert (not Has_Nan (This));
   end Predict;

   procedure Add_Process_Noise (This : in out Core;
                                At_Time : Ada.Real_Time.Time)
   is
      use Ada.Real_Time;
      Interval : constant Duration
        := To_Duration (At_Time - This.Last_Process_Noise_Update_Time);
      Dt : constant Float := Float (Interval);
      Pn : Process_Noise_T renames Process_Noise;
      Tmp : Float;
   begin
      pragma Assert (not Has_Nan (This));
      This.Last_Process_Noise_Update_Time := At_Time;

      Tmp := (Pn.Acc_X_Y * Dt**2 + Pn.Vel * Dt + Pn.Pos)**2;
      This.P (X, X) := This.P (X, X) + Tmp;
      This.P (Y, Y) := This.P (Y, Y) + Tmp;
      This.P (Z, Z) := This.P (Z, Z) + Tmp;

      Tmp := (Pn.Acc_X_Y * Dt + Pn.Vel)**2;
      This.P (Vx, Vx) := This.P (Vx, Vx) + Tmp;
      This.P (Vy, Vy) := This.P (Vy, Vy) + Tmp;
      This.P (Vz, Vz) := This.P (Vz, Vz) + Tmp;

      Tmp := (Pn.Gyro_Roll_Pitch * Dt + Pn.Att)**2;
      This.P (D0, D0) := This.P (D0, D0) + Tmp;
      This.P (D1, D1) := This.P (D1, D1) + Tmp;

      Tmp := (Pn.Gyro_Yaw * Dt + Pn.Att)**2;
      This.P (D2, D2) := This.P (D1, D1) + Tmp;

      --  Enforce symmetry of the covariance matrix, and ensure the
      --  values stay bounded
      for R in This.P'Range (1) loop
         for C in This.P'Range (2) loop
            declare
               P : constant Float := 0.5 * (This.P (R, C) + This.P (C, R));
            begin
               if Is_Nan (P) or else P > Max_Covariance then
                  This.P (R, C) := Max_Covariance;
                  This.P (C, R) := Max_Covariance;
               elsif R = C and P < Min_Covariance then
                  This.P (R, R) := Min_Covariance;
               else
                  This.P (R, C) := P;
                  This.P (C, R) := P;
               end if;
            end;
         end loop;
      end loop;

      pragma Assert (not Has_Nan (This));
   end Add_Process_Noise;

   procedure Scalar_Update (This               : in out Core;
                            Measurement_Vector :        Space_Vector;
                            Error              :        Float;
                            Standard_Deviation :        Float)
   is
      pragma Assert (not Has_Nan (This));

      --  Kalman gain, treated as column vector
      K : Space_Vector;
      --  Innovation covariance
      PHT : constant Space_Vector :=  This.P * Measurement_Vector;
      R : constant Float := Standard_Deviation**2;
      HPHR : Float := R;
   begin
      for J in PHT'Range loop
         HPHR := HPHR + Measurement_Vector (J) * PHT (J);
      end loop;
      pragma Assert (not Is_Nan (HPHR));

      --  XXX We should update the estimate! which will involve x',
      --  x'', dt. And, in general, how do we predict?

      --  Measurement update
      for J in K'Range loop
         K (J) := PHT (J) / HPHR;
         This.S (J) := This.S (J) + K (J) * Error;
      end loop;
      pragma Assert (not Has_Nan (This));

      --  Covariance update

      --  XXX this is the formula in the C source
      declare
         Old_P : constant Space_Matrix := This.P;
         TmpNN1 : Space_Matrix;
      begin
         --  K*H - I
         TmpNN1 := K * Measurement_Vector;
         TmpNN1 := TmpNN1
           - Unit_Matrix
             (TmpNN1'Length (1), TmpNN1'First (1), TmpNN1'First (2));
         --  (K*H - I) * P' * (K*H - I)'
         This.P := TmpNN1 * This.P * Transpose (TmpNN1);
         pragma Assert (not Has_Nan (This));
      end;

      --  XXX This is the formula in the MIT paper; (I - KH) * P
      --  declare
      --     Old_P : constant Space_Matrix := This.P;
      --     TmpNN1 : Space_Matrix;
      --  begin
      --     --  I - K*H
      --     TmpNN1 := Unit_Matrix
      --       (TmpNN1'Length (1), TmpNN1'First (1), TmpNN1'First (2))
      --       - K * Measurement_Vector;
      --     --  (I - K*H) * P'
      --     This.P := TmpNN1 * This.P;
      --     pragma Assert (not Has_Nan (This));
      --  end;

      --  Add the measurement variance and ensure boundedness and
      --  symmetry
      --  (C query: why would the bounds be hit?
      for I in K'Range loop  -- sorry about the I!
         for J in K'Range loop
            declare
               V : constant Float := K (I) * R * K (J);
               P : constant Float
                 := (This.P (I, J) + This.P (J, I)) / 2.0 + V;
            begin
               if Is_Nan (P) or else P > Max_Covariance then
                  This.P (I, J) := Max_Covariance;
                  This.P (J, I) := Max_Covariance;
               elsif I = J and then P < Min_Covariance then
                  This.P (I, J) := Min_Covariance;
               else
                  This.P (I, J) := P;
                  This.P (J, I) := P;
               end if;
            end;
         end loop;
      end loop;
      pragma Assert (not Has_Nan (This));  -- could this happen?

   end Scalar_Update;

   procedure Update_With_Baro (This      : in out Core;
                               Altitude  :        Float)
   is
   begin
      pragma Assert (not Has_Nan (This));

      This.Updated := True;

      if not This.Quad_Is_Flying
        or else This.Barometer_Reference_Height < 1.0
      then
         This.Barometer_Reference_Height := Altitude;
      end if;

      Scalar_Update (This               => This,
                     Measurement_Vector => (Z => 1.0, others => 0.0),
                     Error              =>
                       (Altitude - This.Barometer_Reference_Height)
                         - This.S (Z),
                     Standard_Deviation => Process_Noise.Baro);
      pragma Assert (not Has_Nan (This));
   end Update_With_Baro;

   procedure Update_With_Flow
     (This : in out Core;
      Flow :        Stabilizer_Types.Flow_Measurement;
      Gyro :        Stabilizer_Types.Angular_Velocity_Degrees)
   is
      Degrees_To_Radians : constant := Ada.Numerics.Pi / 180.0;
      --  Camera constants
      --  Maybe to do with aperture?
      Npix : constant := 30.0;
      Thetapix : constant :=  4.0 * Degrees_To_Radians;
      Omega_Factor : constant := 1.25;
      --  Body rates
      Omegax_B : constant Float := Gyro.X * Degrees_To_Radians;
      Omegay_B : constant Float := Gyro.Y * Degrees_To_Radians;
      --  Global rates
      Dx_G : constant Float := This.S (Vx);
      Dy_G : constant Float := This.S (Vy);
      Z_G : constant Float := Float'Max (This.S (Z), 0.1);
   begin
      This.Updated := True;
      --  X velocity prediction and update
      declare
         Predicted_Nx : constant Float
           := Flow.Dt * Npix / Thetapix
             * ((Dx_G * This.R (2, 2) / Z_G) - Omega_Factor * Omegay_B);
      begin
         Scalar_Update
           (This               => This,
            Measurement_Vector =>
              (Z => -Flow.Dt * Npix / Thetapix * This.R (2, 2) * Dx_G / Z_G**2,
               Vx => Flow.Dt * Npix / Thetapix * This.R (2, 2) / Z_G,
               others => 0.0),
            Error              =>
              (Flow.Dx - Predicted_Nx),
            Standard_Deviation => Flow.Standard_Deviation_X);
         pragma Assert (not Has_Nan (This));
      end;
      --  Y velocity prediction and update
      declare
         Predicted_Ny : constant Float
           := Flow.Dt * Npix / Thetapix
             * ((Dy_G * This.R (2, 2) / Z_G) - Omega_Factor * Omegax_B);
      begin
         Scalar_Update
           (This               => This,
            Measurement_Vector =>
              (Z => -Flow.Dt * Npix / Thetapix * This.R (2, 2) * Dy_G / Z_G**2,
               Vy => Flow.Dt * Npix / Thetapix * This.R (2, 2) / Z_G,
               others => 0.0),
            Error              =>
              (Flow.Dy - Predicted_Ny),
            Standard_Deviation => Flow.Standard_Deviation_Y);
         pragma Assert (not Has_Nan (This));
      end;
   end Update_With_Flow;

   procedure Update_With_ToF
     (This : in out Core;
      ToF  :        Stabilizer_Types.ToF_Measurement)
   is
      Z_Scale : Float renames This.R (2, 2);
   begin
      --  Only update the filter if the measurement is reliable.
      if Z_Scale in 0.1 .. 1.0 then
         This.Updated := True;
         declare
            Predicted_Distance : constant Float := This.S (Z) / Z_Scale;
         begin
            Scalar_Update (This               => This,
                           Measurement_Vector =>
                             (Z => 1.0 / Z_Scale, others => 0.0),
                           Error              =>
                             (ToF.Distance - Predicted_Distance),
                           Standard_Deviation => ToF.Standard_Deviation);
            pragma Assert (not Has_Nan (This));
         end;
      end if;
   end Update_With_ToF;

   procedure Finalize (This : in out Core)
   is
      V0 : constant Float := This.S (D0);
      V1 : constant Float := This.S (D1);
      V2 : constant Float := This.S (D2);
   begin
      pragma Assert (not Has_Nan (This));

      if not This.Updated then
         return;
      end if;
      This.Updated := False;

      --  Move attitude error into attitude if any of the angle
      --  errors are large enough (but not too large)
      if (abs V0 > 1.0e-4 or abs V1 > 1.0e-4 or abs V2 > 1.0e-4)
        and (abs V0 < 10.0 and abs V1 < 10.0 and abs V2 < 10.0)
      then
         declare
            Angle : constant Float := Sqrt (V0**2 + V1**2 + V2**2);
            Ca : constant Float := Cos (Angle / 2.0);
            Sa : constant Float := Sin (Angle / 2.0);
            Dq : constant Quaternion
              := (Ca, Sa * V0 / Angle, Sa * V1 / Angle, Sa * V2 / Angle);
            Tq : constant Quaternion
              := (Dq.W * This.Q.W - Dq.X * This.Q.X
                    - Dq.Y * This.Q.Y - Dq.Z * This.Q.Z,
                  Dq.X * This.Q.W + Dq.W * This.Q.X
                    + Dq.Z * This.Q.Y - Dq.Y * This.Q.Z,
                  Dq.Y * This.Q.W - Dq.Z * This.Q.X
                    + Dq.W * This.Q.Y + Dq.X * This.Q.Z,
                  Dq.Z * This.Q.W - Dq.Y * This.Q.X
                    + Dq.X * This.Q.Y + Dq.W * This.Q.Z);
            --  Normalize and store the result
            Norm : constant Float
              := Sqrt (Tq.W**2 + Tq.X**2 + Tq.Y**2 + Tq.Z**2);
         begin
            This.Q.W := Tq.W / Norm;
            This.Q.X := Tq.X / Norm;
            This.Q.Y := Tq.Y / Norm;
            This.Q.Z := Tq.Z / Norm;
         end;

         --  Rotate the covariance, since we've rotated the body
         declare
            --  Matrix to rotate the attitude covariances once updated
            A : Space_Matrix
              := Unit_Matrix (Space_Matrix'Length (1),
                              Space_Matrix'First (1),
                              Space_Matrix'First (2));
            H0 : constant Float := V0 / 2.0;
            H1 : constant Float := V1 / 2.0;
            H2 : constant Float := V2 / 2.0;
         begin
            A (D0, D0) := 1.0 - H1**2 / 2.0 - H2**2 / 2.0;
            A (D0, D1) := H2 + H0 * H1 / 2.0;
            A (D0, D2) := -H1 + H0 * H2 / 2.0;

            A (D1, D0) := -H2 + H0 * H1 / 2.0;
            A (D1, D1) := 1.0 - H0**2 / 2.0 - H2**2 / 2.0;
            A (D1, D2) := H0 + H1 * H2 / 2.0;

            A (D2, D0) := H1 + H0 * H2 / 2.0;
            A (D2, D1) := -H0 + H1 * H2 / 2.0;
            A (D2, D2) := 1.0 - H0**2 / 2.0 - H1**2 / 2.0;

            --  declare
            --     Variation : constant Float := abs (Determinant (A) - 1.0);
            --  begin
            --     pragma Assert (Variation < 1.0e-5);
            --     null;
            --  end;

            This.P := A * This.P * Transpose (A);
         end;
      end if;
      pragma Assert (not Has_Nan (This));

      --  Convert the new attitude to a rotation matrix, such that
      --  we can rotate body-frame velocity and acc
      declare
         R : Real_Matrix renames This.R;
         Q : Quaternion renames This.Q;
      begin
         R (0, 0) := Q.W**2 + Q.X**2 - Q.Y**2 - Q.Z**2;
         R (0, 1) := 2.0 * (Q.X * Q.Y - Q.W * Q.Z);
         R (0, 2) := 2.0 * (Q.X * Q.Z + Q.W * Q.Y);

         R (1, 0) := 2.0 * (Q.X * Q.Y + Q.W * Q.Z);
         R (1, 1) := Q.W**2 - Q.X**2 + Q.Y**2 - Q.Z**2;
         R (1, 2) := 2.0 * (Q.Y * Q.Z - Q.W * Q.X);

         R (2, 0) := 2.0 * (Q.X * Q.Z - Q.W * Q.Y);
         R (2, 1) := 2.0 * (Q.Y * Q.Z + Q.W * Q.X);
         R (2, 2) := Q.W**2 - Q.X**2 - Q.Y**2 + Q.Z**2;

         declare
            Variation : constant Float := abs (Determinant (R) - 1.0);
         begin
            pragma Assert (Variation < 1.0e-5);
            null;
         end;
      end;

      --  Reset the attitude error
      This.S (D0) := 0.0;
      This.S (D1) := 0.0;
      This.S (D2) := 0.0;

      --  Enforce symmetry of the covariance matrix, and ensure the
      --  values stay bounded
      for R in This.P'Range (1) loop
         for C in This.P'Range (2) loop
            declare
               P : constant Float := 0.5 * (This.P (R, C) + This.P (C, R));
            begin
               if Is_Nan (P) or else P > Max_Covariance then
                  This.P (R, C) := Max_Covariance;
                  This.P (C, R) := Max_Covariance;
               elsif R = C and P < Min_Covariance then
                  This.P (R, R) := Min_Covariance;
               else
                  This.P (R, C) := P;
                  This.P (C, R) := P;
               end if;
            end;
         end loop;
      end loop;
      pragma Assert (not Has_Nan (This));

      --  XXX correctness of the covariance matrix is ensured ???
      if not State_Is_Within_Bounds (This) then
         This.Reset_Estimation := True;
      end if;
   end Finalize;

   function State_Is_Within_Bounds (This : Core) return Boolean
   is
      Max_Position : constant := 100.0;  -- m
      Max_Velocity : constant := 10.0;   -- m/s
      Pos, Vel : Boolean;
   begin
      Pos := (for all P in X .. Z   => abs This.S (P) < Max_Position);
      Vel := (for all V in Vx .. Vz => abs This.S (V) < Max_Velocity);
      return Pos and Vel;
   end State_Is_Within_Bounds;

   procedure Externalize_State
     (This    :     Core;
      State   : out Stabilizer_Types.State_Data;
      Acc     :     Stabilizer_Types.Acceleration;
      At_Time :     Ada.Real_Time.Time)
   is
      pragma Unreferenced (At_Time);
      --  The C code uses this (which is a T_Uint32 tick) to set the
      --  timestamps in the components of State. I don't think those
      --  timestamps actually do any good in either Kalman or
      --  SensFusion6 algorithms.

      R : Real_Matrix renames This.R;
      S : Real_Vector renames This.S;
      Q : Quaternion  renames This.Q;
   begin
      pragma Assert (not Has_Nan (This));

      --  Position state is already in world frame
      State.Pos := (Timestamp => 0, X => S (X), Y => S (Y), Z => S (Z));

      --  Velocity is in body frame and needs to be rotated to
      --  world frame
      State.Vel :=
        (Timestamp => 0,
         X => R (0, 0) * S (Vx) + R (0, 1) * S (Vy) + R (0, 2) * S (Vz),
         Y => R (1, 0) * S (Vx) + R (1, 1) * S (Vy) + R (1, 2) * S (Vz),
         Z => R (2, 0) * S (Vx) + R (2, 1) * S (Vy) + R (2, 2) * S (Vz));

      --  Accelerometer measurements are in the body frame and need
      --  to be rotated to world frame.
      --
      --  Furthermore, the legacy code requires acc.z to be
      --  acceleration without gravity.
      --
      --  Finally, note that these accelerations are in Gs, and not
      --  in m/s^2, hence - 1 for removing gravity
      State.Acc :=
        (Timestamp => 0,
         X => (R (0, 0) * Acc.X + R (0, 1) * Acc.Y + R (0, 2) * Acc.Z) / G,
         Y => (R (1, 0) * Acc.X + R (1, 1) * Acc.Y + R (1, 2) * Acc.Z) / G,
         Z => (R (2, 0) * Acc.X + R (2, 1) * Acc.Y + R (2, 2) * Acc.Z) / G
           - 1.0);

      --  Convert the new attitude into Euler YPR
      declare
         Yaw : constant Float
           := Arctan (2.0 * (Q.X * Q.Y + Q.W * Q.Z),
                      Q.W**2 + Q.X**2 - Q.Y**2 - Q.Z**2);
         Pitch : constant Float
           := Arcsin (-2.0 * (Q.X * Q.Z - Q.W * Q.Y));
         Roll : constant Float
           := Arctan (2.0 * (Q.Y * Q.Z + Q.W * Q.X),
                      Q.W**2 - Q.X**2 - Q.Y**2 + Q.Z**2);
         Radians_To_Degrees : constant Float := 180.0 / Ada.Numerics.Pi;
      begin
         State.Att :=
           (Timestamp => 0,
            Roll  => Roll * Radians_To_Degrees,
            Pitch => -Pitch * Radians_To_Degrees,
            Yaw   => Yaw * Radians_To_Degrees);
      end;

      --  Save quaternion, hopefully one day this could be used in
      --  a better controller.
      --
      --  Note that this is not adjusted for the legacy coordinate system
      State.Quat :=
        (Timestamp => 0,
         W         => This.Q.W,
         X         => This.Q.X,
         Y         => This.Q.Y,
         Z         => This.Q.Z);

      pragma Assert (not Has_Nan (This));
   end Externalize_State;

end Kalman_Core;
