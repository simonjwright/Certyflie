with SensFusion6;

package body Estimators.Original is

   procedure Estimate
     (This    : in out Original_Estimator;
      State   : in out State_Data;
      Sensors :    out Sensor_Data;
      --  Control :        Control_Data;
      Tick    :        Types.T_Uint32)
   is
      Delta_T : constant := 1.0 / 250.0;  -- XXX
   begin
      This.Acquire_Sensor_Data (Sensors => Sensors, Tick => Tick);

      SensFusion6.Update_Q (Gx => Sensors.Gyro.X,
                            Gy => Sensors.Gyro.Y,
                            Gz => Sensors.Gyro.Z,
                            Ax => Sensors.Acc.X,
                            Ay => Sensors.Acc.Y,
                            Az => Sensors.Acc.Z,
                            Mx => Sensors.Mag.X,
                            My => Sensors.Mag.Y,
                            Mz => Sensors.Mag.Z,
                            Dt => Delta_T);

      SensFusion6.Get_Euler_RPY (Euler_Roll_Actual => State.Att.Roll,
                                 Euler_Pitch_Actual => State.Att.Pitch,
                                 Euler_Yaw_Actual => State.Att.Yaw);
      State.Att.Timestamp := Tick;

      SensFusion6.Get_Quaternion (Qx => State.Quat.X,
                                  Qy => State.Quat.Y,
                                  Qz => State.Quat.Z,
                                  Qw => State.Quat.W);
      State.Quat.Timestamp := Tick;

      --  update the z velocity; in the C, this happens in
      --  position_estimator_altitude.c!
      --
      --  The deadband is 0.04 G
      --  The alpha is 0.995
      --
      --  Do we need this?

      --  Get the ToF input & do a position_estimate().

   end Estimate;

end Estimators.Original;
