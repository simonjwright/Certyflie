with Types;
with Stabilizer_Types;

package Estimators is

   type Estimator is abstract tagged limited private;

   procedure Initialize (This : in out Estimator)
   is null;

   procedure Enqueue (This : in out Estimator;
                      Flow : Stabilizer_Types.Flow_Measurement) is null;

   procedure Enqueue (This : in out Estimator;
                      ToF  : Stabilizer_Types.ToF_Measurement) is null;

   procedure Estimate
     (This    : in out Estimator;
      State   : in out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32) is abstract;
   --  State should be internal to Estimator, but might imply lots of
   --  interfaces.
   --
   --  Is Sensors actually used outside? - I think so, for Altitude
   --  Hold.
   --
   --  Control isn't used in the complementary estimator.

   type Estimator_P is access all Estimator'Class;

   function Current_Estimator return Estimator_P;

   procedure Set_Required_Estimator (To : in out Estimator'Class);

private

   type Estimator is abstract tagged limited null record;

   procedure Acquire_Sensor_Data (This    :     Estimator;
                                  Sensors : out Stabilizer_Types.Sensor_Data;
                                  Tick    :     Types.T_Uint32);

end Estimators;
