package Kalman_Core.Data is

   type Data_Index is mod 2 ** 3;

   type Predict_Data_Record is record
      Time      : Ada.Real_Time.Time;
      Core_Data : Core;
      Thrust    : Float;
      Acc       : Stabilizer_Types.Acceleration;
      Gyro      : Stabilizer_Types.Angular_Velocity_Radians;
      Interval  : Ada.Real_Time.Time_Span;
      Is_Flying : Boolean;
   end record;
   Predict_Data : array (Data_Index) of Predict_Data_Record;
   Predict_Data_Index : Data_Index := Data_Index'Last;

   type Flow_Data_Record is record
      Time : Ada.Real_Time.Time;
      Flow : Stabilizer_Types.Flow_Measurement;
      Gyro : Stabilizer_Types.Angular_Velocity_Degrees;
   end record;
   Flow_Data : array (Data_Index) of Flow_Data_Record;
   Flow_Data_Index : Data_Index := Data_Index'Last;

   type Tof_Data_Record is record
      Time : Ada.Real_Time.Time;
      ToF  : Stabilizer_Types.ToF_Measurement;
   end record;
   Tof_Data : array (Data_Index) of Tof_Data_Record;
   Tof_Data_Index : Data_Index := Data_Index'Last;

   type Final_Data_Record is record
      Time : Ada.Real_Time.Time;
      Data : Core;
   end record;
   Final_Data : array (Data_Index) of Final_Data_Record;
   Final_Data_Index : Data_Index := Data_Index'Last;

end Kalman_Core.Data;
