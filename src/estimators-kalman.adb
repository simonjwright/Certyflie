with Ada.Numerics;
with Ada.Real_Time;

package body Estimators.Kalman is

   --  The idea is that Estimate gets called at 500 Hz, reading sensor
   --  data, but the major work of prediction is done less often.
   Prediction_Update_Interval : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (4);  -- 250 Hz

   --  Likewise for barometer readings.
   Baro_Update_Interval : constant Ada.Real_Time.Time_Span
     := Ada.Real_Time.Milliseconds (40);  -- 25 Hz

   Next_Prediction_Time : Ada.Real_Time.Time;
   Next_Baro_Update_Time : Ada.Real_Time.Time;

   Is_Flying : Boolean := False;  -- XXX need Situation Awareness!

   procedure Initialize (This : in out Kalman_Estimator)
   is
      Now : constant Ada.Real_Time.Time := Ada.Real_Time.Clock;
   begin
      Kalman_Core.Initialize (This.Core, At_Time => Now);
      Next_Prediction_Time := Now;
      Next_Baro_Update_Time := Now;
   end Initialize;

   procedure Estimate
     (This    : in out Kalman_Estimator;
      State   : in out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32)
   is
      Now : constant Ada.Real_Time.Time := Ada.Real_Time.Clock;
      use type Ada.Real_Time.Time;
   begin
      This.Acquire_Sensor_Data (Sensors => Sensors, Tick => Tick);

      --  Run the system dynamics to predict the state forward, every
      --  other call (250 Hz reduced from 500 Hz, nominally)
      if Now >= Next_Prediction_Time then
         declare
            G : constant Float := 9.81;  -- m/s^2
            CF_Weight : constant := 27.0;
            --  Full-scale corresponds to 60 g of thrust. It seems.
            Thrust_To_Acceleration : constant Float
              := G * 60.0 / CF_Weight / 65536.0;
            Thrust : constant Float
              := Float (Control.Thrust) * Thrust_To_Acceleration;
            In_Flight_Thrust_Threshold : constant := G * 0.1; -- WTF

            Degrees_To_Radians : constant := Ada.Numerics.Pi / 180.0;
            Gyro_In_MKS : constant  Stabilizer_Types.Vector_3
              := (Sensors.Gyro.Timestamp,
                  Sensors.Gyro.X * Degrees_To_Radians,
                  Sensors.Gyro.Y * Degrees_To_Radians,
                  Sensors.Gyro.Z * Degrees_To_Radians);
            Acc_In_MKS : constant Stabilizer_Types.Vector_3
              := (Sensors.Acc.Timestamp,
                  Sensors.Acc.X * G,
                  Sensors.Acc.Y * G,
                  Sensors.Acc.Z * G);
         begin
            --  Assume that the flight begins when the thrust is large
            --  enough and for now we never stop "flying".
            if Thrust > In_Flight_Thrust_Threshold then
               Is_Flying := True;
            end if;

            Kalman_Core.Predict (This      => This.Core,
                                 Thrust    => Thrust,
                                 Acc       => Acc_In_MKS,
                                 Gyro      => Gyro_In_MKS,
                                 Interval  => Prediction_Update_Interval,
                                 Is_Flying => Is_Flying);
         end;
         Next_Prediction_Time := Now + Prediction_Update_Interval;
      end if;

      --  Add process noise every loop, rather than every prediction.
      Kalman_Core.Add_Process_Noise
        (This => This.Core, At_Time => Now);

      --  --  Use the barometer measurements if it's time
      --  if Now >= Next_Baro_Update_Time then
      --     Kalman_Core.Update_With_Baro
      --       (This      => This.Core,
      --        Altitude  => Sensors.Baro.Altitude);
      --     Next_Baro_Update_Time := Now + Baro_Update_Interval;
      --  end if;

      --  Reads the Gyro data again???

      declare
         Flow : Stabilizer_Types.Flow_Measurement;
         Valid : Boolean;
      begin
         This.Flow.Get (Flow, Valid);
         if Valid then
            Kalman_Core.Update_With_Flow (This => This.Core,
                                          Flow => Flow,
                                          Gyro => Sensors.Gyro);
         end if;
      end;

      declare
         ToF : Stabilizer_Types.ToF_Measurement;
         Valid : Boolean;
      begin
         This.ToF.Get (ToF, Valid);
         if Valid then
            Kalman_Core.Update_With_ToF (This => This.Core,
                                         ToF  => ToF);
         end if;
      end;

      --  If an update has been made, the state is finalized:
      --  - the attitude error is moved into the body attitude quaternion,
      --  - the body attitude is converted into a rotation matrix for the
      --    next prediction, and
      --  - correctness of the covariance matrix is ensured
      Kalman_Core.Finalize (This.Core);

      Kalman_Core.Externalize_State (This.Core,
                                     State => State,
                                     Acc => Sensors.Acc,
                                     At_Time => Now);

   end Estimate;

end Estimators.Kalman;
