private with Kalman_Core;

package Estimators.Kalman is

   type Kalman_Estimator is new Estimator with private;

   overriding
   procedure Initialize (This : in out Kalman_Estimator);

   overriding
   procedure Estimate
     (This    : in out Kalman_Estimator;
      State   : in out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32);

private

   type Kalman_Estimator is new Estimator with record
      Core : Kalman_Core.Core;
   end record;

end Estimators.Kalman;
