------------------------------------------------------------------------------
--                              Certyflie                                   --
--                                                                          --
--                     Copyright (C) 2015-2018, AdaCore                     --
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

with Ada.Interrupts.Names;
with System;
with System.Storage_Elements;

with Crazyflie_Config;
with HAL;
with STM32.Device;
with STM32.DMA;
with STM32.DMA.Interrupts;
with STM32.EXTI;
with STM32.USARTs;
with STM32.Board;
with STM32.GPIO;

with Generic_Queue;

with LEDS;

package body UART_Syslink is

   Red_L   : LEDS.Flasher (LEDS.Red_L'Access);
   --  Indicate there's been an overrun on receive.

   Red_R   : LEDS.Flasher (LEDS.Red_R'Access);
   --  Indicate that a DMA transfer has been suspended for flow
   --  control.

   --  Declarations that used to be in the private part of the spec.

   package T_Uint8_Queue is new Generic_Queue (Types.T_Uint8);
   use T_Uint8_Queue;

   --  Global variables and constants

   Controller : STM32.DMA.DMA_Controller renames STM32.Device.DMA_2;

   Tx_Channel : constant STM32.DMA.DMA_Channel_Selector := STM32.DMA.Channel_5;
   Tx_Stream : constant STM32.DMA.DMA_Stream_Selector := STM32.DMA.Stream_6;

   UART_RX_QUEUE_SIZE   : constant := 256;
   --  UART_DATA_TIMEOUT_MS : constant Time_Span :=  Milliseconds (1_000);

   --  Procedures and functions

   --  Initialize the STM32F4 USART controller.
   procedure Initialize_USART;

   --  Configure the STM32F4 USART controller.
   procedure Configure_USART;

   --  Initialize the STM32F4 DMA controller.
   procedure Initialize_DMA;

   --  Initialize NRF flow control.
   procedure Initialize_Flow_Control;

   --  Enable USART interrupts in reception.
   procedure Enable_USART_Rx_Interrupts;

   --  Tasks and protected objects

   Tx_IRQ_Handler : STM32.DMA.Interrupts.DMA_Interrupt_Controller
     (Controller => STM32.Device.DMA_2'Access,
      Stream     => STM32.DMA.Stream_6,
      ID         => Ada.Interrupts.Names.DMA2_Stream6_Interrupt,
      Priority   => Crazyflie_Config.HIGH_INTERRUPT_PRIORITY);

   --  EXTI4 Interrupt Handler for transmission flow control.
   protected Flow_Control_Handler is
      pragma Interrupt_Priority (Crazyflie_Config.TOP_INTERRUPT_PRIORITY);

      procedure Starting_Transfer (Number_Of_Bytes : Natural;
                                   Starting_From   : System.Address);
   private
      --  for pause/resume
      Pausing             : Boolean        := False;
      Remaining_Transfers : HAL.UInt16     := 0;
      First_Past_Buffer   : System.Address := System.Null_Address;
      --  If we have to resume a transfer, we start from (this address
      --  - the number of transfers remaining).

      procedure Pause_Transfer;

      procedure Resume_Transfer;

      procedure Flow_Control_IRQ_Handler;
      pragma Attach_Handler (Flow_Control_IRQ_Handler,
                             Ada.Interrupts.Names.EXTI4_Interrupt);
   end Flow_Control_Handler;

   --  Interrupt Handler for reception (DMA not used here).
   protected Rx_IRQ_Handler is
      pragma Interrupt_Priority (Crazyflie_Config.SYSLINK_INTERRUPT_PRIORITY);

      entry Await_Byte_Reception (Rx_Byte : out Types.T_Uint8);

   private

      Byte_Avalaible  : Boolean := False;
      Rx_Queue        : T_Queue (UART_RX_QUEUE_SIZE);

      procedure IRQ_Handler;
      pragma Attach_Handler (IRQ_Handler,
                             Ada.Interrupts.Names.USART6_Interrupt);
   end Rx_IRQ_Handler;

   --  Public procedures and functions

   ----------
   -- Init --
   ----------

   procedure Init is
   begin
      Initialize_USART;
      Configure_USART;
      Initialize_DMA;
      Initialize_Flow_Control;

      STM32.USARTs.Enable (STM32.Board.NRF_USART);

      Enable_USART_Rx_Interrupts;
   end Init;

   -----------------------
   -- Get_Data_Blocking --
   -----------------------

   procedure Get_Data_Blocking (Rx_Byte : out Types.T_Uint8) is
   begin
      Rx_IRQ_Handler.Await_Byte_Reception (Rx_Byte);
   end Get_Data_Blocking;

   ----------------------------
   -- Send_DMA_Data_Blocking --
   ----------------------------

   procedure Send_DMA_Data_Blocking
     (Data_Size : Natural;
      Data      : DMA_Data) is
      Result : STM32.DMA.DMA_Error_Code;
   begin
      Flow_Control_Handler.Starting_Transfer
        (Number_Of_Bytes => Data_Size,
         Starting_From   => Data'Address);

      Tx_IRQ_Handler.Start_Transfer
        (Source      => Data'Address,
         Destination =>
           STM32.USARTs.Data_Register_Address (STM32.Board.NRF_USART),
         Data_Count  => HAL.UInt16 (Data_Size));
      --  The Crazyflie C code only enables Transfer_Complete
      --  interrupt. Presumably they just stumble on if something goes
      --  wrong.

      STM32.USARTs.Clear_Status (STM32.Board.NRF_USART,
                                 STM32.USARTs.Transmission_Complete_Indicated);
      STM32.USARTs.Enable_DMA_Transmit_Requests (STM32.Board.NRF_USART);

      Tx_IRQ_Handler.Wait_For_Completion (Status => Result);

      case Result is
         when STM32.DMA.DMA_No_Error | STM32.DMA.DMA_FIFO_Error =>
            null;
         when others =>
            raise Program_Error with Result'Img;
      end case;
   end Send_DMA_Data_Blocking;

   --  Private procedures and functions

   ----------------------
   -- Initialize_USART --
   ----------------------

   procedure Initialize_USART
   is
   begin
      STM32.Device.Enable_Clock (Points => (STM32.Board.NRF_RX,
                                            STM32.Board.NRF_TX));

      STM32.GPIO.Configure_IO
        (STM32.Board.NRF_RX,
         Config => (Mode             => STM32.GPIO.Mode_AF,
                    AF               => STM32.Board.NRF_USART_AF,
                    AF_Output_Type   => STM32.GPIO.Open_Drain,
                    AF_Speed         => STM32.GPIO.Speed_25MHz,
                    Resistors        => STM32.GPIO.Pull_Up));

      STM32.GPIO.Configure_IO
        (STM32.Board.NRF_TX,
         Config => (Mode             => STM32.GPIO.Mode_AF,
                    AF               => STM32.Board.NRF_USART_AF,
                    AF_Output_Type   => STM32.GPIO.Push_Pull,
                    AF_Speed         => STM32.GPIO.Speed_25MHz,
                    Resistors        => STM32.GPIO.Pull_Up));

      STM32.Device.Enable_Clock (STM32.Board.NRF_USART);
   end Initialize_USART;

   ---------------------
   -- Configure_USART --
   ---------------------

   procedure Configure_USART is
      use STM32.Board;
      use STM32.USARTs;
   begin
      Disable (NRF_USART);

      Set_Baud_Rate    (NRF_USART, 1_000_000);
      Set_Mode         (NRF_USART, Tx_Rx_Mode);
      Set_Stop_Bits    (NRF_USART, Stopbits_1);
      Set_Word_Length  (NRF_USART, Word_Length_8);
      Set_Parity       (NRF_USART, No_Parity);
      Set_Flow_Control (NRF_USART, No_Flow_Control);

      Enable (NRF_USART);
   end Configure_USART;

   --------------------
   -- Initialize_DMA --
   --------------------

   procedure Initialize_DMA is
   begin
      STM32.Device.Enable_Clock (Controller);

      STM32.DMA.Configure
        (Controller,
         Stream => Tx_Stream,
         Config =>
           (Channel                      => Tx_Channel,
            Direction                    => STM32.DMA.Memory_To_Peripheral,
            Increment_Peripheral_Address => False,
            Increment_Memory_Address     => True,
            Peripheral_Data_Format       => STM32.DMA.Bytes,
            Memory_Data_Format           => STM32.DMA.Bytes,
            Operation_Mode               => STM32.DMA.Normal_Mode,
            Priority                     => STM32.DMA.Priority_High,
            FIFO_Enabled                 => False,
            others                       => <>));
      --  note the controller is disabled by the call to Configure
   end Initialize_DMA;

   -----------------------------
   -- Initialize_Flow_Control --
   -----------------------------

   procedure Initialize_Flow_Control is
   begin
      STM32.Device.Enable_Clock (STM32.Board.NRF_FLOW_CTRL);

      STM32.GPIO.Configure_IO (STM32.Board.NRF_FLOW_CTRL,
                               Config => (Mode      => STM32.GPIO.Mode_In,
                                          Resistors => STM32.GPIO.Pull_Up));

      STM32.EXTI.Clear_External_Interrupt
        (STM32.GPIO.Interrupt_Line_Number (STM32.Board.NRF_FLOW_CTRL));

      STM32.GPIO.Configure_Trigger
        (STM32.Board.NRF_FLOW_CTRL,
         Trigger => STM32.EXTI.Interrupt_Rising_Falling_Edge);
   end Initialize_Flow_Control;

   --------------------------------
   -- Enable_USART_Rx_Interrupts --
   --------------------------------

   procedure Enable_USART_Rx_Interrupts is
   begin
      STM32.USARTs.Enable_Interrupts
        (STM32.Board.NRF_USART,
         Source => STM32.USARTs.Received_Data_Not_Empty);
   end Enable_USART_Rx_Interrupts;

   --  Tasks and protected objects

   --------------------------
   -- Flow_Control_Handler --
   --------------------------

   protected body Flow_Control_Handler is

      procedure Starting_Transfer (Number_Of_Bytes : Natural;
                                   Starting_From   : System.Address) is
         pragma Assert (not Pausing, "starting_transfer w/ pausing");
         use System.Storage_Elements;
      begin
         First_Past_Buffer := Starting_From + Storage_Offset (Number_Of_Bytes);
      end Starting_Transfer;

      procedure Pause_Transfer is
      begin
         if not Pausing then
            if STM32.DMA.Enabled (Controller, Tx_Stream) then
               STM32.DMA.Disable_Interrupt
                 (Controller,
                  Tx_Stream,
                  STM32.DMA.Transfer_Complete_Interrupt);
               STM32.DMA.Disable (Controller, Tx_Stream);
               --  Waits til current tx done.

               STM32.DMA.Clear_Status
                 (Controller,
                  Tx_Stream,
                  STM32.DMA.Transfer_Complete_Indicated);
               Remaining_Transfers :=
                 STM32.DMA.Current_NDT (Controller, Tx_Stream);
               --  Flash the right red LED.
               Red_R.Set;
               Pausing := True;
            end if;
         end if;
      end Pause_Transfer;

      procedure Resume_Transfer is
         use System.Storage_Elements;
      begin
         if Pausing then
            Pausing := False;
            STM32.DMA.Set_NDT (Controller, Tx_Stream, Remaining_Transfers);
            STM32.DMA.Set_Memory_Buffer
              (Controller,
               Tx_Stream,
               STM32.DMA.Memory_Buffer_0,
               First_Past_Buffer - Storage_Offset (Remaining_Transfers));
            STM32.DMA.Enable_Interrupt
              (Controller, Tx_Stream, STM32.DMA.Transfer_Complete_Interrupt);
            STM32.USARTs.Clear_Status
              (STM32.Board.NRF_USART,
               STM32.USARTs.Transmission_Complete_Indicated);
            STM32.DMA.Enable (Controller, Tx_Stream);
         end if;
      end Resume_Transfer;

      procedure Flow_Control_IRQ_Handler is
      begin
         STM32.EXTI.Clear_External_Interrupt
           (STM32.GPIO.Interrupt_Line_Number (STM32.Board.NRF_FLOW_CTRL));
         --  See RM 10.3.14, DMA Transfer Suspension
         if STM32.GPIO.Set (STM32.Board.NRF_FLOW_CTRL) then
            Pause_Transfer;
         else
            Resume_Transfer;
         end if;
      end Flow_Control_IRQ_Handler;

   end Flow_Control_Handler;

   --------------------
   -- Rx_IRQ_Handler --
   --------------------

   protected body Rx_IRQ_Handler is

      --------------------------
      -- Await_Byte_Reception --
      --------------------------

      entry Await_Byte_Reception (Rx_Byte : out Types.T_Uint8)
        when Byte_Avalaible is
      begin
         Dequeue (Rx_Queue, Rx_Byte);
         Byte_Avalaible := not Is_Empty (Rx_Queue);
      end Await_Byte_Reception;

      -----------------
      -- IRQ_Handler --
      -----------------

      procedure IRQ_Handler is
         Received_Byte : Types.T_Uint8;
         use type HAL.UInt9;
      begin
         if STM32.USARTs.Status (STM32.Board.NRF_USART,
                                 STM32.USARTs.Read_Data_Register_Not_Empty)
         then
            Received_Byte :=
              Types.T_Uint8
                (STM32.USARTs.Current_Input (STM32.Board.NRF_USART)
                 and 16#FF#);
            STM32.USARTs.Clear_Status
              (STM32.Board.NRF_USART,
               STM32.USARTs.Read_Data_Register_Not_Empty);
            Enqueue (Rx_Queue, Received_Byte);
            Byte_Avalaible := True;
         elsif STM32.USARTs.Status (STM32.Board.NRF_USART,
                                    STM32.USARTs.Overrun_Error_Indicated)
         then
            --  Flash the left red LED.
            Red_L.Set;
            --  RM0090 rev11 top of p 1001: overrun is cleared by
            --  reading SR (which we've already done (twice, now?!)),
            --  then reading DR.
            --
            --  We got here because the DR was empty. The incoming
            --  character (the one that didn't make it from the shift
            --  register to the DR) is discarded. We assume the
            --  missing data can be recovered by higher-level
            --  protocols.
            Received_Byte :=
              Types.T_Uint8 (STM32.USARTs.Current_Input (STM32.Board.NRF_USART)
                             and 16#FF#);
            STM32.USARTs.Clear_Status
              (STM32.Board.NRF_USART,
               STM32.USARTs.Read_Data_Register_Not_Empty);
            STM32.USARTs.Clear_Status
              (STM32.Board.NRF_USART,
               STM32.USARTs.Overrun_Error_Indicated);
         end if;
      end IRQ_Handler;

   end Rx_IRQ_Handler;

begin
   STM32.USARTs.Disable_Interrupts
     (STM32.Board.NRF_USART,
      Source => STM32.USARTs.Received_Data_Not_Empty);
   STM32.USARTs.Disable_DMA_Transmit_Requests (STM32.Board.NRF_USART);
end UART_Syslink;
