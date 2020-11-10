------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--        Copyright (C) 2020, Simon Wright <simon@pushface.org>             --
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

package Deck is

   procedure Init;

   function Test return Boolean;

   --  I'm not going to get too clever here. I think the decks are
   --  actually free-running; so the flow deck just tells the
   --  estimator (who he?) what it has measured.

   --  function Count return Natural;

   --  function Driver_Count return Natural;

   --  function Get_Required_Estimator return State_Estimator_Type;
   --  XXX surely this is driver-dependent?

   --  function Get_Required_Low_Interference_Radio_Mode return Boolean;
   --  XXX surely this is driver-dependent?

end Deck;
