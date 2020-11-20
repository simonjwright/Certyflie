------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--           Copyright (C) 2020, Simon Wright <simon@pushface.org>          --
--                                                                          --
--  This library is free software;  you can redistribute it and/or modify   --
--  it under terms of the  GNU General Public License  as published by the  --
--  Free Software  Foundation;  either version 3,  or (at your  option) any --
--  later version. This library is distributed in the hope that it will be  --
--  useful, but WITHOUT ANY WARRANTY;  without even the implied warranty of --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                    --
--                                                                          --
--  As a special exception under Section 7 of GPL version 3, you are        --
--  granted additional permissions described in the GCC Runtime Library     --
--  Exception, version 3.1, as published by the Free Software Foundation.   --
--                                                                          --
--  You should have received a copy of the GNU General Public License and   --
--  a copy of the GCC Runtime Library Exception along with this program;    --
--  see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see   --
--  <http://www.gnu.org/licenses/>.                                         --
--                                                                          --
--  As a special exception, if other files instantiate generics from this   --
--  unit, or you link this unit with other files to produce an executable,  --
--  this  unit  does not  by itself cause  the resulting executable to be   --
--  covered by the GNU General Public License. This exception does not      --
--  however invalidate any other reasons why the executable file  might be  --
--  covered by the  GNU Public License.                                     --
------------------------------------------------------------------------------

--  See LPS25H Datasheet DocID023722 Rev 6

with HAL.I2C;
with HAL.Time;

package LPS25H.I2C is

   type LPS25H_Barometric_Sensor_I2C
     (Port              : not null HAL.I2C.Any_I2C_Port;
      Slave_Address_Odd : Boolean;
      Timing            : not null HAL.Time.Any_Delays)
     is new LPS25H_Barometric_Sensor with private;
   --  See Datasheet section 5.2.1 for slave address adjustment

   procedure Initialize (This : in out LPS25H_Barometric_Sensor_I2C);

   procedure Get_Data
     (This   : in out LPS25H_Barometric_Sensor_I2C;
      Press  :    out Pressure;
      Temp   :    out Temperature;
      Asl    :    out Altitude;
      Status :    out Boolean);

private

   type LPS25H_Barometric_Sensor_I2C
     (Port              : not null HAL.I2C.Any_I2C_Port;
      Slave_Address_Odd : Boolean;
      Timing            : not null HAL.Time.Any_Delays)
      is new LPS25H_Barometric_Sensor with record
         I2C_Address : HAL.I2C.I2C_Address;
      end record;

end LPS25H.I2C;
