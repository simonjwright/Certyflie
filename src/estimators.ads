with Ada.Real_Time;

package Estimators is

   --  There are several estimators in the Crazyflie stabilizer
   --  subsystem.
   --
   --  Each of them accepts data from a subset of the available
   --  sources. These inputs are provided to be queued for the current
   --  estimator; if the data isn't required, it will be dropped.

   --  Only the data sources provided by the Flow Deck are processed
   --  so far (3.vi.20).

   --  Flow (horizontal movement over ground)
   type Flow_Measurement is record
      Timestamp            : Ada.Real_Time.Time;
      Dx                   : Float;
      Dy                   : Float;
      Standard_Deviation_X : Float;
      Standard_Deviation_Y : Float;
      Dt                   : Float;
   end record;

   --  Height
   type ToF_Measurement is record
      Timestamp          : Ada.Real_Time.Time;
      Distance           : Float;
      Standard_Deviation : Float;
   end record;

   type Estimator is tagged limited private;

   procedure Enqueue (E : in out Estimator; Flow : Flow_Measurement) is null;

   procedure Enqueue (E : in out Estimator; ToF : ToF_Measurement) is null;

   type Estimator_P is access all Estimator'Class;

   function Current_Estimator return Estimator_P;

   procedure Set_Required_Estimator (To : in out Estimator'Class);

private

   type Estimator is tagged limited null record;

end Estimators;
