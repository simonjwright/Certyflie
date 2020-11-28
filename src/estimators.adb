with IMU;

package body Estimators is

   Current : Estimator_P;

   function Current_Estimator return Estimator_P is (Current);

   procedure Set_Required_Estimator (To : in out Estimator'Class) is
   begin
      Current := To'Unchecked_Access;
   end Set_Required_Estimator;

   procedure Acquire_Sensor_Data (This    :     Estimator;
                                  Sensors : out Sensor_Data;
                                  Tick    :     Types.T_Uint32)
   is
      Gyro        : IMU.Gyroscope_Data;
      Acc         : IMU.Accelerometer_Data;
      Mag         : IMU.Magnetometer_Data;
   begin
      IMU.Read_9 (Gyro, Acc, Mag);
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
