with Ada.Real_Time;
with Types;

package Estimators is

   --  There are several estimators in the Crazyflie stabilizer
   --  subsystem.
   --
   --  Each of them accepts data from a subset of the available
   --  sources. These inputs are provided to be queued for the current
   --  estimator; if the data isn't required, it will be dropped.

--  /** Attitude in euler angle form */
--  typedef struct attitude_s {
--    uint32_t timestamp;  // Timestamp when the data was computed

--    float roll;
--    float pitch;
--    float yaw;
--  } attitude_t;

   --  Attitude in Euler angle form.

   subtype Angle is Float;

   type Attitude is record
      Timestamp : Types.T_Uint32;
      Roll      : Angle;
      Pitch     : Angle;
      Yaw       : Angle;
   end record;

--  struct vec3_s {
--    uint32_t timestamp; // Timestamp when the data was computed

--    float x;
--    float y;
--    float z;
--  };

   type Vector_3 is record
      Timestamp : Types.T_Uint32;
      X         : Float := 0.0;
      Y         : Float := 0.0;
      Z         : Float := 0.0;
   end record;

--  typedef struct vec3_s vector_t;
--  typedef struct vec3_s point_t;
--  typedef struct vec3_s velocity_t;
--  typedef struct vec3_s acc_t;

   --  XXX This needs reconsidering!
   subtype Vector                   is Vector_3;
   subtype Point                    is Vector_3;
   subtype Velocity                 is Vector_3;
   subtype Acceleration             is Vector_3;  -- G
   subtype Angular_Velocity_Degrees is Vector_3;
   subtype Angular_Velocity_Radians is Vector_3;
   subtype Magnetometer_Output      is Vector_3;  -- microTeslas

--  typedef struct quaternion_s {
--    uint32_t timestamp;

--    union {
--      struct {
--        float q0;
--        float q1;
--        float q2;
--        float q3;
--      };
--      struct {
--        float x;
--        float y;
--        float z;
--        float w;
--      };
--    };
--  } quaternion_t;

   subtype Quaternion_Component is Float range -1.0 .. 1.0;
   type Quaternion is record
      Timestamp : Types.T_Uint32;
      X : Quaternion_Component := 0.0;
      Y : Quaternion_Component := 0.0;
      Z : Quaternion_Component := 0.0;
      W : Quaternion_Component := 0.0;
   end record;

--  typedef struct sensorData_s {
--    Axis3f acc;               // Gs
--    Axis3f gyro;              // deg/s
--    Axis3f mag;               // gauss
--    baro_t baro;
--  #ifdef LOG_SEC_IMU
--    Axis3f accSec;            // Gs
--    Axis3f gyroSec;           // deg/s
--  #endif
--    uint64_t interruptTimestamp;
--  } sensorData_t;

   type Barometer_Data is record
      Pressure    : Float;
      Temperature : Float;
      Altitude    : Float;
   end record;

   type Sensor_Data is record
      Timestamp : Types.T_Uint32;
      Acc       : Acceleration;             -- G
      Gyro      : Angular_Velocity_Degrees; -- degrees/sec
      Mag       : Magnetometer_Output;      -- ? gauss
      Baro      : Barometer_Data;
   end record;

   --  Only the additional data sources provided by the Flow Deck are
   --  processed so far (21.xi.20).

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

--  typedef struct state_s {
--    attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
--    quaternion_t attitudeQuaternion;
--    point_t position;         // m
--    velocity_t velocity;      // m/s
--    acc_t acc;                // Gs (but acc.z without considering gravity)
--  } state_t;

   type State_Data is record
      Att  : Attitude;     -- degrees, pitch inverted (legacy CF2 body system)
      Quat : Quaternion;
      Pos  : Point;        -- m
      Vel  : Velocity;     -- m/s
      Acc  : Acceleration; -- G (but Z component doesn't consider gravity)
   end record;

   type Estimator is abstract tagged limited private;

   procedure Enqueue (This : in out Estimator;
                      Flow : Flow_Measurement) is null;

   procedure Enqueue (This : in out Estimator;
                      ToF  : ToF_Measurement) is null;

   procedure Estimate
     (This    : in out Estimator;
      State   : in out State_Data;
      Sensors :    out Sensor_Data;
      --  Control :        Control_Data;
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
                                  Sensors : out Sensor_Data;
                                  Tick    :     Types.T_Uint32);

end Estimators;
