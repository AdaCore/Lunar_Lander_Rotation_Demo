------------------------------------------------------------------------------
--                                                                          --
--                    Copyright (C) 2017, AdaCore                           --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of the copyright holder nor the names of its     --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
------------------------------------------------------------------------------

--  This program uses I2C to communicate with the BNO055 sensor.
--
--  This is the main procedure for a program that interacts with the AdaFruit
--  BNO055 breakout board in order to send orientation data to a host computer.
--  The host displays a 3D model via the "Processing" application (see below),
--  and the rotation data received from this program controls the rotation of
--  the model display in real time.
--
--  Rotation data are sent to the host via serial connection. You can use a
--  USB cable specifically designed to appear as a COM port, for example
--  from Mouser (among others):
--
--  * Mouser Part No:       895-TTL-232R-5V
--  * Manufacturer Part No: TTL-232R-5V
--  * Manufacturer:         FTDI
--
--  but note that the above is a 5-volt cable in case that is an issue. There
--  are 3-volt versions available as well.
--
--  The end of the cable is a female header, described in the datasheet
--  (`DS_TTL-232R_CABLES-217672.pdf`).  See pages 10 and 11 in particular.
--
--  Header pin 4 on the cable is TXD, the transmit data output.
--  Header pin 5 on the cable is RXD, the receive data input.
--
--  Connect the cable header pins to the STMicro board GPIO pins as follows:
--
--  * header pin 1, the black wire's header slot, to a ground pin on the board
--  * header pin 4, the orange wire's header slot, to PB7
--  * header pin 5, the yellow wire's header slot, to PB6
--
--  On the host, just plug in the USB-to-serial cable. Once the cable is
--  connected it will appear like a host serial port and is to be selected within
--  the Processing app displaying the model.
--
--  The USART is set to: 115_200, N81, no flow control, as expected by the
--  Processing app.
--
--  The model is displayed using the Processing app, freely available here:
--
--     https://processing.org/
--
--  You must download and install this Processing app on the host computer.
--  The app runs programs -- "sketches" -- in source files with the "pde"
--  extension. Once installed, you can invoke Processing by double-clicking
--  (say) on the specific sketch file that displays the model and listens to
--  the serial port. This program file is named "lander.pde" and is located
--  in a subdirectory of the "processing/" dir included with this Ada program.
--  Within the Processing app, with this sketch file loaded, press the "Run"
--  icon to start the "lander" sketch execution. It does not matter
--  whether you start the program on the ST Micro board before or after the
--  "lander" program. Once "lander" is running in its separate window,
--  select the host port that the ST Micro board is connected to via the cable
--  described above. Now the displayed model will reflect the rotation data
--  coming from the BNO055 via the ST Micro board and the serial connection.
--
--  Within the "lander" execution window there is a checkbox that enables
--  display of the incoming serial data. These data are displayed in the
--  Processing app, not in the sketch window. You can use this display to
--  verify data are coming from the ST Micro board. Note that calibration data
--  are included, and that the model display will behave best once the gyro and
--  accelerometer are both calibrated. You will need to manually calibrate the
--  IMU board as usual, i.e., by physically moving it in space.
--
--  NOTE: if the sketch running in the Processing app is unresponsive, there
--  may be multiple instances of "java.exe" running (on Windows). If so, shut
--  down Processing app, kill all these java executables, and restart the
--  Processing app (and then press the Run button to run the sketch).
--
--  This demo is based on one provided by AdaFruit:
--  https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
--
--  The model we use is available here:
--  https://github.com/nasa/NASA-3D-Resources/tree/master/3D%20Models/Apollo%20Lunar%20Module

with Last_Chance_Handler;  pragma Unreferenced (Last_Chance_Handler);

with HAL; use HAL;

with BNO055_I2C;  use BNO055_I2C;  --  the BNO055 with comms based on I2C

with Peripherals;        use Peripherals;
with Serial_IO.Blocking; use Serial_IO.Blocking;
with Message_Buffers;    use Message_Buffers;
use Serial_IO;
with Floating_Point_Utilities;

with STM32.Device;  use STM32.Device;
with STM32.Board;   use STM32.Board;
with STM32.GPIO;    use STM32.GPIO;
with STM32.I2C;     use STM32.I2C;

use STM32; -- for GPIO_Alternate_Function

with Ada.Real_Time; use Ada.Real_Time;

procedure Lunar_Lander_Demo is

   IMU : BNO055_9DOF_IMU (Sensor_Port'Access);

   Calibration : Calibration_States;

   Data : Sensor_Data;

   Outgoing : aliased Message (Physical_Size => 1024);  -- arbitrary size
   --  Used to contain the strings sent to the host

   procedure Put (This : String);
   --  Sends the input string over the serial port.

   procedure Put (This : Float);
   --  Sends the image of the input value over the serial port, using a utility
   --  function that removes any leading blanks in that image.

   procedure New_Line;
   --  Sends a CR/LF character sequence over the serial port. You can chnage
   --  this to just a LF if desired.

   procedure Set_Up_IMU;
   --  Performs all hardware initialization for the STM32 board to work with
   --  the BNO055 breakout board and then resets the breakout board using the
   --  selected Reset pin. Raises Program_Error if the device does not indicate
   --  that it is a BNO055 sensor.

   procedure Initialize_IMU_IO_Hardware
     (Port   : access I2C_Port;
      SCL    : GPIO_Point;
      SCL_AF : GPIO_Alternate_Function;
      SDA    : GPIO_Point;
      SDA_AF : GPIO_Alternate_Function;
      Reset  : GPIO_Point);

   procedure Reset_BNO055_Via_Hardware (Reset : in out GPIO_Point);

   ---------
   -- Put --
   ---------

   procedure Put (This : String) is
   begin
      Set (Outgoing, To => This);
      Put (Host, Outgoing'Unchecked_Access);
   end Put;

   -----------------
   -- Float_Utils --
   -----------------

   package Float_Utils is new Floating_Point_Utilities (Float);

   ---------
   -- Put --
   ---------

   procedure Put (This : Float) is
      Stripped_Image : constant String := Float_Utils.Image_of (This);
      --  Because at most one blank may appear between values, we make sure
      --  that Put does not include a leading blank via the Image_of function.
   begin
      Set (Outgoing, To => Stripped_Image);
      Put (Host, Outgoing'Unchecked_Access);
   end Put;

   --------------
   -- New_Line --
   --------------

   procedure New_Line is
   begin
      Set (Outgoing, To => ASCII.CR & ASCII.LF);
      Put (Host, Outgoing'Unchecked_Access);
   end New_Line;

   --------------------------------
   -- Initialize_IMU_IO_Hardware --
   --------------------------------

   procedure Initialize_IMU_IO_Hardware
     (Port   : access I2C_Port;
      SCL    : GPIO_Point;
      SCL_AF : GPIO_Alternate_Function;
      SDA    : GPIO_Point;
      SDA_AF : GPIO_Alternate_Function;
      Reset  : GPIO_Point)
   is
      GPIO_Conf            : GPIO_Port_Configuration;
      Selected_Clock_Speed : constant := 10_000;
   begin
      Enable_Clock (SCL);
      Enable_Clock (SDA);
      Enable_Clock (Reset);

      Enable_Clock (Port.all);

      STM32.Device.Reset (Port.all);

      GPIO_Conf.Speed       := Speed_25MHz;
      GPIO_Conf.Mode        := Mode_Out;
      GPIO_Conf.Output_Type := Push_Pull;
      GPIO_Conf.Resistors   := Floating;
      Configure_IO (Reset, GPIO_Conf);

      Configure_Alternate_Function (SCL, SCL_AF);
      Configure_Alternate_Function (SDA, SDA_AF);

      GPIO_Conf.Speed       := Speed_100MHz;
      GPIO_Conf.Mode        := Mode_AF;
      GPIO_Conf.Output_Type := Open_Drain;
      GPIO_Conf.Resistors   := Pull_Up;
      Configure_IO (SCL, GPIO_Conf);
      Configure_IO (SDA, GPIO_Conf);

      STM32.I2C.Configure
        (Port.all,
         (Clock_Speed              => Selected_Clock_Speed,
          Addressing_Mode          => Addressing_Mode_7bit,
          General_Call_Enabled     => False,
          Clock_Stretching_Enabled => True,
          Own_Address              => 16#00#,
          others                   => <>));

      Set_State (Port.all, Enabled => True);
   end Initialize_IMU_IO_Hardware;

   -------------------------------
   -- Reset_BNO055_Via_Hardware --
   -------------------------------

   procedure Reset_BNO055_Via_Hardware (Reset : in out GPIO_Point) is
   begin
      --  reset is active low
      STM32.GPIO.Clear (Reset);
      --  the BNO055 Datasheet, section 3.2, says 20ns is required
      delay until Clock + Milliseconds (1);
      STM32.GPIO.Set (Reset);
      delay until Clock + Milliseconds (650);  --  essential
      --  the time required after reset is the Power On Reset (POR) time
      --  specified in the Datasheet, table 0-2, "From Reset to Config mode"
   end Reset_BNO055_Via_Hardware;

   ----------------
   -- Set_Up_IMU --
   ----------------

   procedure Set_Up_IMU is
   begin
      Initialize_IMU_IO_Hardware
        (Port   => Selected_I2C_Port,
         SCL    => Selected_I2C_Clock_Pin,
         SCL_AF => Selected_I2C_Port_AF,
         SDA    => Selected_I2C_Data_Pin,
         SDA_AF => Selected_I2C_Port_AF,
         Reset  => Selected_HW_Reset_Pin);

      Reset_BNO055_Via_Hardware (Selected_HW_Reset_Pin);

      if IMU.Device_Id /= I_Am_BNO055 then
         raise Program_Error with "No BNO055!";
      end if;
   end Set_Up_IMU;

begin
   Initialize_LEDs;

   Initialize (Host);
   Configure (Host, Baud_Rate => 115_200);
   --  Note the configuration must match the Processing app sketch settings

   Set_Up_IMU;

   IMU.Configure;

   loop
      Data := IMU.Output (Kind => Euler_Orientation);

      --  NB: AT MOST ONE blank may appear between values! The proceure Put
      --  that takes a floating point argument ensures that no extra blanks
      --  are output.
      Put ("Orientation: ");
      Put (Data (X));
      Put (" ");
      Put (Data (Y));
      Put (" ");
      Put (Data (Z));
      New_Line;

      Calibration := IMU.Sensor_Calibration;

      --  NB: AT MOST ONE blank may appear between values! These are unsigned
      --  values so we use the automatic blank insertion from 'Img
      Put ("Calibration:");
      Put (Calibration.Platform'Img);
      Put (Calibration.Gyroscope'Img);
      Put (Calibration.Accelerometer'Img);
      Put (Calibration.Magnetometer'Img);
      Put (" "); --  required for correct parsing in model display
      New_Line;

      Board.Toggle (Green_LED);  --  to further indicate we are running

      delay until Clock + Milliseconds (BNO055_Min_Sample_Interval);
   end loop;
end Lunar_Lander_Demo;
