# Embedded project. "Smart car".

## Brief
* Project built with PlatformIO framework.
* Project contains both Arduino mega and ESP8266 code.
* ESP8266 code runs on Arduino framework.
* If you want to upload code in 1 command, you need to specify upload_port for mega2560-master.
* You need to upload data to esp if you want to communicate with Arduino board remotely.

## Build
### GUI
Press **build** and then **upload**

### Terminal
Write in terminal
```sh
pio start
```

### Work with car remotely
You need to upload **data** directory in the root of the project to ESP8266.

Write the following in terminal to do so.
```sh
pio run -t uploadfs
```

After that you will need to upload code to ESP8266. Once done, open serial monitor and copy printed IP address.

Then you can insert it in your browser and you are ready to control the car from that page.

## Car functionality
* Display cm traveled and joystick horizontal position on LCD.
* Ride forward, backward with joystick.
* Toggle motors power on joystick button press.
* Work with 3 different commands from UART (http server included).
    * dst:dst_val <- command to drive car dst_val cm.
    * rot:rot_val <- command to rotate car rot_val degress.
    * vel:vel_val <- command to set car motors' power. **Attention!** To use that you need to make sure joystick button pressed and no coordinate mappings happen.