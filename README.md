# Demo for the Bosch 9DOF IMU driving a 3-D model of the lunar lander

IMPORTANT: Please note that this project exists as part of a blog entry,
article, or other similar material by AdaCore. It is provided for
convenient access the software described therein. As such, it is not
updated regularly and may, over time, become inconsistent with the
library or libraries it utilizes (for example, the Ada Drivers Library).

This is a demonstration program that interacts with the AdaFruit BNO055 
breakout board in order to send orientation data to a host computer.The 
host displays a 3D model of the Apollo Lunar Excursion Module (LEM) via 
the "Processing" application (see below). The rotation data received 
from the target computer controls the rotation of the model display in 
real time. 

Rotation data are sent to the host via serial connection. You can use a
USB cable specifically designed to appear as a COM port, for example
from Mouser (among others):

* Mouser Part No:       895-TTL-232R-5V
* Manufacturer Part No: TTL-232R-5V
* Manufacturer:         FTDI

but note that the above is a 5-volt cable in case that is an issue. There
are 3-volt versions available as well.

The end of the cable is a female header, described in the datasheet
(`DS_TTL-232R_CABLES-217672.pdf`).  See pages 10 and 11 in particular.

Header pin 4 on the cable is TXD, the transmit data output.
Header pin 5 on the cable is RXD, the receive data input.

Connect the cable header pins to the STMicro board GPIO pins as follows:

* header pin 1, the black wire's header slot, to a ground pin on the board
* header pin 4, the orange wire's header slot, to PC7
* header pin 5, the yellow wire's header slot, to PC6

On the host, just plug in the USB-to-serial cable. Once the cable is
connected it will appear like a host serial port and is to be selected within
the Processing app displaying the model.

The USART is set to: 115_200, N81, no flow control, as expected by the
Processing app.

The model is displayed using the Processing app, freely available here:

   https://processing.org/

You must download and install this Processing app on the host computer.
The app runs programs -- "sketches" -- in source files with the "pde"
extension. Once installed, you can invoke Processing by double-clicking
(say) on the specific sketch file that displays the model and listens to
the serial port. This program file is named "lander.pde" and is located
in a subdirectory of the "processing/" dir included with this Ada program.
Within the Processing app, with this sketch file loaded, press the "Run"
icon to start the "lander" sketch execution. It does not matter
whether you start the program on the ST Micro board before or after the
"lander" program. Once "lander" is running in its separate window,
select the host port that the ST Micro board is connected to via the cable
described above. Now the displayed model will reflect the rotation data
coming from the BNO055 via the ST Micro board and the serial connection.

Within the "lander" execution window there is a checkbox that enables
display of the incoming serial data. These data are displayed in the
Processing app, not in the sketch window. You can use this display to
verify data are coming from the ST Micro board. Note that calibration data
are included, and that the model display will behave best once the gyro and
accelerometer are both calibrated. You will need to manually calibrate the
IMU board as usual, i.e., by physically moving it in space.

NOTE: if the sketch running in the Processing app is unresponsive, there
may be multiple instances of "java.exe" running (on Windows). If so, shut
down Processing app, kill all these java executables, and restart the
Processing app (and then press the Run button to run the sketch).

This demo is based on one provided by AdaFruit:
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview

The model we use is available here:
https://github.com/nasa/NASA-3D-Resources/tree/master/3D%20Models/Apollo%20Lunar%20Module

