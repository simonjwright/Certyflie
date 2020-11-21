package body Estimators is

   Null_Estimator : aliased Estimator;
   Current : Estimator_P := Estimator'Class (Null_Estimator)'Access;

   function Current_Estimator return Estimator_P is (Current);

   procedure Set_Required_Estimator (To : in out Estimator'Class) is
   begin
      Current := To'Unchecked_Access;
   end Set_Required_Estimator;

end Estimators;
