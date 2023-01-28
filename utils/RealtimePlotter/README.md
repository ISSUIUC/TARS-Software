RealtimePlotter
===============

This is a realtime plotting utility written in Processing that is really useful in debugging.

This was not written by ISS, but only modified to suit our needs better. Credit goes to [sebnil/RealtimePlotter](https://github.com/sebnil/RealtimePlotter)

Update as of 2022: We have now written functionality to use the Telemetry Ground Station GUI as a serial plotter by printing the same JSON string that the feather on the ground prints through serial in flight software. This can be switched on and off using the `SERIAL_PLOTTING` ifdef in `telemetry.cpp`.

Debugging sensors on a microprocessor can be a hassle and the most used approach is to output the sensor values to a serial monitor. I made a realtime plotting tool which is a better and more visual way of doing the same thing.

![Realtime plotter](http://sebastiannilsson.com/wp-content/uploads/2013/12/RealtimePlotterProcessing-300x216.png)

I created this software to debug an Arduino Due on my self-balancing robot. To tune the controls of the robot I needed fast feedback to know if I was making progress or not. The video below demonstrates typical use of the realtime plotter:
## Video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=WN2oXuavrWk
" target="_blank"><img src="http://img.youtube.com/vi/WN2oXuavrWk/0.jpg" 
alt="Realtime plotter video" width="240" height="180" border="10" /></a>

- Real-time plotter of your data while it is still being processed by your application
- Plots live data from serial port. Microprocessor choice does not matter as long as it can send serial data to your computer.
- 6 channels of data (and this can be increased if necessary)
- Live bar charts
- Live line graphs
- You just send the data you want to debug with a space as delimiter like this "value1 value2 value3 value4 value5 value6". Floats or integers does not matter.
- Open source
- Robust. It will not crash because of corrupt data stream or similar.
- Multi platform Java. Tested on OSX and Windows 8 (and should work on Linux as well).

More info and other projects at http://sebastiannilsson.com/en/projekt/realtime-plotter/realtime-data-plotter/

### How to install and use
Since I have an Arduino I will use it as example but any micro processor can be used.

1. Get [ProcessingIDE](http://processing.org/) to run the code. It is a neat and useful IDE for doing graphical stuff.
3. Connect the Arduino to the usb or serial port of your computer.
4. Upload the example code (RealtimePlotterArduinoCode) to the Arduino
5. Check serial monitor (at 115200) and check that it outputs data in the format "value1 value2 value3 value4 value5 value6\r". Always end with a line break. Another way to do formatting is with the printf function like so: printf("%u %u %u %u %u %u\r", data1, data2,...); 
6. Close the serial monitor (since only one resource can use the serial port at the same time).
7. Open the Processing sketch (Either BasicRealtimePlotter/BasicRealtimePlotter.pde or RealtimePlotterWithControlPanel/RealtimePlotterWithControlPanel.pde)
8. In the Processing IDE, go to Sketch->Import Library. Search for ControlP5 from Andreas Schlegel and Install it. (I used version 2.2.6). You can also manually download [controlP5 gui library](http://www.sojamo.de/libraries/controlP5/) and unzip it into your Processing libraries folder

9. Edit the sketch file so that the serial port name to correspond to the actual port ("COM3", "COM5", "/dev/tty.usbmodem1411" or whatever you have). (If you want to test the code without a serial port just set boolean mockupSerial = true;)
10. Run the sketch


### Advanced use
The realtime plotter can be expanded to also send commands to the microprocessor. The usual approach when programming microprocessors is to set some parameters in the beginning of the code, upload them to the processor, see the result, change the parameters again, upload, and so on until satisfactory performance is achieved. This iterative process takes a lot of time and a better approach is to send updated parameters to the microprocessor from your computer via serial data. For example I needed to tune some parameters on my robot and created a command panel that runs in parallell with the realtime plotter. For each change in parameters I immediately am able to see the result on the plotting screen. Example code of this is located in /RealtimePlotterWithControlPanel.

![RealtimePlotterProcessingWithControlPanel](http://sebastiannilsson.com/wp-content/uploads/2013/12/RealtimePlotterProcessingWithControlPanel-300x140.png)

### Notes
I decided to send and receive the data as ascii characters instead of binaries. The greatest disadvantage is performance. The greatest advantage is ease of use.

In some sense the realtime data plotter can also be used as a very slow and limited digital oscilloscope. I would not recommend using it for any high frequency applications though.

### Processing 2/3 compatability
- For Processing 3 compatability, use the most recent code.
- For Processing 2 compatability, use the code from the Processing-2 branch or [download the older release](https://github.com/sebnil/RealtimePlotter/releases/tag/1.0).

### Some comments about earlier approaches and the used libraries
I have tried many different ways of doing this. My first approach was Matlab but I had problems with it locking the serial port. It was a hassle to get it working and getting everything configured takes to much time. My second approach was Python and graphing libraries but this was still not very satisfactory. The Processing language together with a graph library and ControlP5 made the whole thing much easier.


## Support me creating open source software:
[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=sebnil&url=https://github.com/sebnil/RealtimePlotter)

<a href='https://ko-fi.com/A0A2HYRH' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=0' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>
