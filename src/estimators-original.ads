--  This is called Complementary in the C sources. They _may_ have
--  meant "complimentary", i.e. free?

package Estimators.Original is

   type Original_Estimator is new Estimator with private;

   overriding
   procedure Estimate
     (This    : in out Original_Estimator;
      State   : in out State_Data;
      Sensors :    out Sensor_Data;
      --  Control :        Control_Data;
      Tick    :        Types.T_Uint32);

   --  We're not interested in Flow or ToF measurements.

private

   type Original_Estimator is new Estimator with null record;

end Estimators.Original;
