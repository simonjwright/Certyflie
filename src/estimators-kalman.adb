with Ada.Numerics;
with Ada.Real_Time;

with Config;

package body Estimators.Kalman is

   --  Estimate is called at high priority. The complicated
   --  calculations are done in a lower-priority task, to give
   --  precedence to client comms.

   task Kalman_Task
     with
       Priority             => Config.KALMAN_TASK_PRIORITY,
       Secondary_Stack_Size => 5120; -- XXX guess

   --  There's a little dance to communicate with the task.

   --  First, it needs access to the Kalman Estimator. There's only
   --  one. Initialize puts a link to it in this PO.
   protected Estimator_Access is
      entry Wait_For_Estimator (The_Estimator : out Estimator_P);
      procedure Set_Estimator (The_Estimator : Estimator_P);
   private
      The_Estimator : Estimator_P;
      Available : Boolean := False;
   end Estimator_Access;

   --  After acquiring the sensor data, Estimate passes it to the
   --  Kalman_Task via this PO.
   protected New_Data is
      entry Wait (Sensors : out Stabilizer_Types.Sensor_Data;
                  Control : out Stabilizer_Types.Control_Data);
      procedure Put (Sensors : Stabilizer_Types.Sensor_Data;
                     Control : Stabilizer_Types.Control_Data);
   private
      The_Sensor_Data : Stabilizer_Types.Sensor_Data;
      The_Control_Data : Stabilizer_Types.Control_Data;
      Available : Boolean := False;
   end New_Data;

   --  Estimate then estimates and updates the State, which is placed
   --  here to be returned to Estimate's caller. Note, it will
   --  probably still be the result of the last run.
   protected Updated_State is
      procedure Put (State : Stabilizer_Types.State_Data);
      function Get return Stabilizer_Types.State_Data;
   private
      The_Data : Stabilizer_Types.State_Data;
   end Updated_State;

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

      Estimator_Access.Set_Estimator (This'Unchecked_Access);
   end Initialize;

   procedure Estimate
     (This    : in out Kalman_Estimator;
      State   :    out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32)
   is
   begin
      This.Acquire_Sensor_Data (Sensors => Sensors, Tick => Tick);

      New_Data.Put (Sensors, Control);

      State := Updated_State.Get;
   end Estimate;

   -- Protected, task bodies --

   protected body Estimator_Access is
      entry Wait_For_Estimator (The_Estimator : out Estimator_P)
        when Available
      is
      begin
         The_Estimator := Estimator_Access.The_Estimator;
         Available := False;
      end Wait_For_Estimator;

      procedure Set_Estimator (The_Estimator : Estimator_P)
      is
      begin
         Estimator_Access.The_Estimator := The_Estimator;
         Available := True;
      end Set_Estimator;
   end Estimator_Access;

   protected body New_Data is
      entry Wait (Sensors : out Stabilizer_Types.Sensor_Data;
                  Control : out Stabilizer_Types.Control_Data)
        when Available
      is
      begin
         Sensors := The_Sensor_Data;
         Control := The_Control_Data;
         Available := False;
      end Wait;

      procedure Put (Sensors : Stabilizer_Types.Sensor_Data;
                     Control : Stabilizer_Types.Control_Data)
      is
      begin
         The_Sensor_Data := Sensors;
         The_Control_Data := Control;
         Available := True;
      end Put;
   end New_Data;

   protected body Updated_State is
      procedure Put (State : Stabilizer_Types.State_Data)
      is
      begin
         The_Data := State;
      end Put;

      function Get return Stabilizer_Types.State_Data
      is (The_Data);
   end Updated_State;

   task body Kalman_Task is
      The_Estimator : Estimator_P;
   begin
      Estimator_Access.Wait_For_Estimator (The_Estimator);

      declare
         This : Kalman_Estimator renames Kalman_Estimator (The_Estimator.all);
         Sensors : Stabilizer_Types.Sensor_Data;
         Control : Stabilizer_Types.Control_Data;
         State : Stabilizer_Types.State_Data;
         Now : Ada.Real_Time.Time;
         use type Ada.Real_Time.Time;
      begin
         loop
            New_Data.Wait (Sensors, Control);
            Now := Ada.Real_Time.Clock;
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
            --  XXX What difference does this make?
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
            --  - the attitude error is moved into the body attitude
            --  - quaternion,
            --  - the body attitude is converted into a rotation
            --    matrix for the next prediction, and
            --  - correctness of the covariance matrix is ensured
            Kalman_Core.Finalize (This.Core);

            Kalman_Core.Externalize_State (This.Core,
                                           State => State,
                                           Acc => Sensors.Acc,
                                           At_Time => Now);
            --  do hairy stuff

            Updated_State.Put (State);
         end loop;
      end; -- declare
   end Kalman_Task;

end Estimators.Kalman;
