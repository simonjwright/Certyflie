------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2017, AdaCore                     --
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

package body LEDS is

   procedure Cancel_Animation (Anim : in out Animation);
   procedure Activate_Animation (Anim : in out Animation);

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      STM32.Board.Initialize_LEDs;
      Is_Initialized := True;
   end Init;

   ----------
   -- Test --
   ----------

   function Test return Boolean is
   begin
      return Is_Initialized;
   end Test;

   ---------
   -- Set --
   ---------

   procedure Set (LED : in out Crazyflie_LED; Value : Boolean)
   is
   begin
      if Value then
         STM32.Board.Turn_On (LED);
      else
         STM32.Board.Turn_Off (LED);
      end if;
   end Set;

   ----------------------
   -- Cancel_Animation --
   ----------------------

   procedure Cancel_Animation (Anim : in out Animation)
   is
      Cancelled : Boolean with Unreferenced;
   begin
      if Anim.Blink_Period > 0.0 then
         Anim.Cancel_Handler (Cancelled);
      end if;

      Set (Anim.LED, False);
   end Cancel_Animation;

   ------------------------
   -- Activate_Animation --
   ------------------------

   procedure Activate_Animation (Anim : in out Animation)
   is
      use type Ada.Real_Time.Time;
   begin
      Set (Anim.LED, True);

      if Anim.Blink_Period > 0.0 then
         Anim.Set_Handler
           (Ada.Real_Time.Clock
              + Ada.Real_Time.To_Time_Span (Anim.Blink_Period),
            Status_Event_Handler.Toggle_Status'Access);
      end if;
   end Activate_Animation;

   ----------------------
   -- Set_System_State --
   ----------------------

   procedure Set_System_State (State : Valid_System_State)
   is
   begin
      if Current_System_Status = State then
         return;
      end if;

      if Current_System_Status /= Initial_State then
         Cancel_Animation (System_Animations (Current_System_Status));
      end if;

      if State in Ready .. Connected then
         if Current_Link_Status /= Connected then
            Current_System_Status := Ready;
         else
            Current_System_Status := Connected;
         end if;
      else
         Current_System_Status := State;
      end if;

      Activate_Animation (System_Animations (Current_System_Status));
   end Set_System_State;

   --------------------
   -- Set_Link_State --
   --------------------

   procedure Set_Link_State (State : Valid_Link_State)
   is
   begin
      if Current_Link_Status = State then
         return;
      end if;

      Current_Link_Status := State;

      if Current_System_Status in Ready .. Connected then
         Set_System_State ((if State = Connected then Connected else Ready));
      end if;
   end Set_Link_State;

   -----------------------
   -- Set_Battery_State --
   -----------------------

   procedure Set_Battery_State (State : Valid_Battery_State)
   is
   begin
      if Current_Battery_Status = State then
         return;
      end if;

      if Current_Battery_Status /= Initial_State then
         Cancel_Animation (Battery_Animations (Current_Battery_Status));
      end if;

      Current_Battery_Status := State;
      Activate_Animation (Battery_Animations (Current_Battery_Status));
   end Set_Battery_State;

   -----------------------
   -- Set_Battery_Level --
   -----------------------

   procedure Set_Battery_Level (Level : Natural)
   is
   begin
      Battery_Animations (On_Battery).Blink_Period :=
        0.5 * Duration (Level);
   end Set_Battery_Level;

   ----------------------
   -- Get_System_State --
   ----------------------

   function Get_System_State return System_State is (Current_System_Status);

   -----------------------
   -- Get_Battery_State --
   -----------------------

   function Get_Battery_State return Battery_State is (Current_Battery_Status);

   --------------------------
   -- Status_Event_Handler --
   --------------------------

   protected body Status_Event_Handler is

      procedure Toggle_Status (Event : in out ARTTE.Timing_Event) is
         Anim : Animation renames
                       Animation (ARTTE.Timing_Event'Class (Event));
         --  We "know" we have an Animation value for the actual parameter
         --  but the formal gives a view of type Timing_Event, so we convert
         --  to the subclass to change the view. (The inner conversion to
         --  the classwide base type is required.) Changing the view allows
         --  reference to the LED and Blink_Period components within Event.
         Next  : Ada.Real_Time.Time;
         use type Ada.Real_Time.Time;
      begin
         Toggle (Anim.LED);

         --  Special case: the LED remains off not as long as it is on.
         if Anim.Blink_Period > 0.5
           and then not Is_Set (Anim.LED)
         then
            Next := Ada.Real_Time.Clock + Ada.Real_Time.Milliseconds (500);

         else
            Next := Ada.Real_Time.Clock
              + Ada.Real_Time.To_Time_Span (Anim.Blink_Period);
         end if;

         --  Set this procedure as the handler for the next occurrence for
         --  Event, too.
         Anim.Set_Handler (Next, Toggle_Status'Access);
      end Toggle_Status;

   end Status_Event_Handler;

   protected Flasher_Handler is
      pragma Interrupt_Priority;
      --  GNAT GPL 2017/ravenscar-full-stm32f4 needs this.

      procedure Turn_Off_The_LED
        (Event : in out ARTTE.Timing_Event);
      --  We "know" that the Event is actually a Flasher.
   end Flasher_Handler;

   procedure Set (The_Flasher : in out Flasher) is
      use type Ada.Real_Time.Time;
   begin
      --  cancel the timing event, if any
      declare
         Dummy : Boolean;
      begin
         The_Flasher.Cancel_Handler (Cancelled => Dummy);
      end;
      --  set the LED
      Set (The_Flasher.The_LED.all, True);
      --  set the timing event to turn off the LED in 5 ms
      The_Flasher.Set_Handler
        (Handler => Flasher_Handler.Turn_Off_The_LED'Access,
         At_Time => Ada.Real_Time.Clock + Ada.Real_Time.Milliseconds (5));
   end Set;

   protected body Flasher_Handler is
      procedure Turn_Off_The_LED
        (Event : in out ARTTE.Timing_Event) is
         The_Flasher : Flasher
           renames Flasher
           (ARTTE.Timing_Event'Class (Event));
      begin
         Set (The_Flasher.The_LED.all, False);
      end Turn_Off_The_LED;
   end Flasher_Handler;

end LEDS;
