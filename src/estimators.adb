with IMU;
with Ada.Numerics.Elementary_Functions;
with Semihosting;

package body Estimators is

   protected body Protected_Flow is
      procedure Put (Flow  : Stabilizer_Types.Flow_Measurement) is
      begin
         Value := Flow;
         Updated := True;
      end Put;
      procedure Get (Flow  : out Stabilizer_Types.Flow_Measurement;
                     Valid : out Boolean) is
      begin
         Flow := Value;
         Valid := Updated;
         Updated := False;
      end Get;
   end Protected_Flow;

   protected body Protected_ToF is
      procedure Put (ToF   : Stabilizer_Types.ToF_Measurement) is
      begin
         Value := ToF;
         Updated := True;
      end Put;
      procedure Get (ToF   : out Stabilizer_Types.ToF_Measurement;
                     Valid : out Boolean) is
      begin
         ToF := Value;
         Valid := Updated;
         Updated := False;
      end Get;
   end Protected_ToF;

   Current : Estimator_P;

   function Current_Estimator return Estimator_P is (Current);

   procedure Set_Required_Estimator (To : in out Estimator'Class) is
   begin
      Current := To'Unchecked_Access;
   end Set_Required_Estimator;

   package Accelerometer_Scaling is
      procedure Process (Acc : in out IMU.Accelerometer_Data);
   end Accelerometer_Scaling;
   package body Accelerometer_Scaling is
      Reported : Boolean := False;
      Accumulated : Float := 0.0;
      Run_In : Natural := 64;
      Count : Natural := 0;
      Limit : constant := 64;
      Scale : Float := 1.0;

      procedure Process (Acc : in out IMU.Accelerometer_Data) is
      begin
         if Run_In > 0 then
            Run_In := Run_In - 1;
            return;
         end if;
         if Reported then
            Acc.X := Acc.X * Scale;
            Acc.Y := Acc.Y * Scale;
            Acc.Z := Acc.Z * Scale;
            return;
         end if;
         Count := Count + 1;
         Accumulated :=
           Accumulated
             + Ada.Numerics.Elementary_Functions.Sqrt
               (Acc.X**2 + Acc.Y**2 + Acc.Z**2);
         if Count = Limit then
            Reported := True;
            Scale := Float (Count) / Accumulated;
            Semihosting.Log_Line ("accel scale: " & Scale'Image);
         end if;
      end Process;

   end Accelerometer_Scaling;

   procedure Enqueue (This : in out Estimator;
                      Flow : Stabilizer_Types.Flow_Measurement)
   is
   begin
      This.Flow.Put (Flow);
   end Enqueue;

   procedure Enqueue (This : in out Estimator;
                      ToF  : Stabilizer_Types.ToF_Measurement)
   is
   begin
      This.ToF.Put (ToF);
   end Enqueue;

   procedure Acquire_Sensor_Data (This    :     Estimator;
                                  Sensors : out Stabilizer_Types.Sensor_Data;
                                  Tick    :     Types.T_Uint32)
   is
      Gyro        : IMU.Gyroscope_Data;
      Acc         : IMU.Accelerometer_Data;
      Mag         : IMU.Magnetometer_Data;
   begin
      IMU.Read_9 (Gyro, Acc, Mag);
      Accelerometer_Scaling.Process (Acc);
      Sensors.Timestamp := Tick;
      Sensors.Acc       := (Timestamp   => Tick,
                            X           => Acc.X,
                            Y           => Acc.Y,
                            Z           => Acc.Z);
      Sensors.Gyro      := (Timestamp   => Tick,
                            X           => Gyro.X,
                            Y           => Gyro.Y,
                            Z           => Gyro.Z);
      Sensors.Mag       := (Timestamp   => Tick,
                            X           => Mag.X,
                            Y           => Mag.Y,
                            Z           => Mag.Z);
      Sensors.Baro      := (Pressure    => 1_000.0,  -- defaults
                            Temperature => 25.0,
                            Altitude    => 0.0);
      if IMU.Has_Barometer then
         declare
            Pressure    : IMU.T_Pressure    := 1_000.0;
            Temperature : IMU.T_Temperature := 25.0;
            Altitude    : IMU.T_Altitude    := 0.0;
            Status      : Boolean;
         begin
            IMU.Read_Barometer_Data (Press  => Pressure,
                                     Temp   => Temperature,
                                     Asl    => Altitude,
                                     Status => Status);
            if Status then
               Sensors.Baro := (Pressure    => Pressure,
                                Temperature => Temperature,
                                Altitude    => Altitude);
            end if;
         end;
      end if;
   end Acquire_Sensor_Data;

end Estimators;
