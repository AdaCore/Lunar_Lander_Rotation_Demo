------------------------------------------------------------------------------
--                                                                          --
--                  Copyright (C) 2015-2016, AdaCore                        --
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
--     3. Neither the name of STMicroelectronics nor the names of its       --
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

--  This package provides the object declarations required for the serial port
--  and the I2C port by the application to communicate with the host and BNO055
--  sensor, respectively.

with STM32.Device;        use STM32.Device;
with Serial_IO.Blocking;  use Serial_IO.Blocking;
use Serial_IO;

with STM32.GPIO;    use STM32.GPIO;
with BNO055_I2C_IO;
with STM32.I2C;     use STM32.I2C;
with BNO055_I2C;    use BNO055_I2C;

use STM32;

package Peripherals is

   --  The specific USART selection is arbitrary.
   Peripheral_For_Host : aliased Serial_IO.Peripheral_Descriptor :=
                  (Transceiver    => USART_6'Access,
                   Transceiver_AF => GPIO_AF_8_USART6,
                   Tx_Pin         => PC6,
                   Rx_Pin         => PC7);

   --  The serial port connected to the host computer, to send data from
   --  the IMU to the host (via the STM32 board) for display.
   Host : Serial_Port (Peripheral_For_Host'Access);


   --  The following are the selections for the specific I2C port and
   --  pins to use to connect to the AdaFruit BNO055 breakout board. These
   --  correspond to the wiring between the breakout board and the STM32
   --  board used. The choice of I2C port is arbitrary, but the GPIO ports
   --  and pins (and alternate function) must correspond to the chosen I2C
   --  port.
   Selected_I2C_Port      : constant access I2C_Port := I2C_2'Access;
   Selected_I2C_Port_AF   : constant GPIO_Alternate_Function := GPIO_AF_4_I2C2;
   Selected_I2C_Clock_Pin : GPIO_Point renames PB10;
   Selected_I2C_Data_Pin  : GPIO_Point renames PB11;
   Selected_HW_Reset_Pin  : GPIO_Point renames PB5; --  PB4;

   --  The bundled I2C port and I2C device address pair, used to communicate
   --  with the BNO055 sensor.
   Sensor_Port : aliased BNO055_I2C_IO.IO_Port := (Selected_I2C_Port, BNO055_Primary_Address);

end Peripherals;
