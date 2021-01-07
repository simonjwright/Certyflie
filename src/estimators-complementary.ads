package Estimators.Complementary is

   type Complementary_Estimator is new Estimator with private;

   overriding
   procedure Estimate
     (This    : in out Complementary_Estimator;
      State   :    out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32);

   --  We're not interested in Flow or ToF measurements (the current C
   --  sources _do_ support ToF).

private

   type Complementary_Estimator is new Estimator with null record;

end Estimators.Complementary;
