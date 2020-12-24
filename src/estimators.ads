with Types;
with Stabilizer_Types;

package Estimators is

   type Estimator is abstract tagged limited private;

   procedure Initialize (This : in out Estimator)
   is null;

   procedure Enqueue (This : in out Estimator;
                      Flow : Stabilizer_Types.Flow_Measurement);

   procedure Enqueue (This : in out Estimator;
                      ToF  : Stabilizer_Types.ToF_Measurement);

   procedure Estimate
     (This    : in out Estimator;
      State   : in out Stabilizer_Types.State_Data;
      Sensors :    out Stabilizer_Types.Sensor_Data;
      Control :        Stabilizer_Types.Control_Data;
      Tick    :        Types.T_Uint32) is abstract;

   type Estimator_P is access all Estimator'Class;

   function Current_Estimator return Estimator_P;

   procedure Set_Required_Estimator (To : in out Estimator'Class);

private

   protected type Protected_Flow is
      procedure Put (Flow  : Stabilizer_Types.Flow_Measurement);
      procedure Get (Flow  : out Stabilizer_Types.Flow_Measurement;
                     Valid : out Boolean);
   private
      Updated : Boolean := False;
      Value   : Stabilizer_Types.Flow_Measurement;
   end Protected_Flow;

   protected type Protected_ToF is
      procedure Put (ToF   : Stabilizer_Types.ToF_Measurement);
      procedure Get (ToF   : out Stabilizer_Types.ToF_Measurement;
                     Valid : out Boolean);
   private
      Updated : Boolean := False;
      Value   : Stabilizer_Types.ToF_Measurement;
   end Protected_ToF;

   type Estimator is abstract tagged limited record
      Flow : Protected_Flow;
      ToF  : Protected_ToF;
   end record;

   procedure Acquire_Sensor_Data (This    :     Estimator;
                                  Sensors : out Stabilizer_Types.Sensor_Data;
                                  Tick    :     Types.T_Uint32);

end Estimators;
