# InDex_Glove
//Codes Related to InDex Project Glove

These codes are used to connect the ESP32 main board to a ROS server, by publishing ROS topics of the connected sensors, via WiFi

The code is an arduino code. 
To add the ESP32 board to arduino you should do the following: 


**************************************************************
                        ARDUINO IDE
**************************************************************
In arduino IDE, goto file ---> preferences ---> additional boards manager url, and add the following: 

http://download.dfrobot.top/FireBeetle/package_esp32_index.json
or this one:
https://dl.espressif.com/dl/package_esp32_index.json

Next goto Tools --> board --> boards manager , and search for ESP32 and install it 



**************************************************************
                     Arduino Libraries
**************************************************************
You should copy the following libraries from the current repo to your arduino library folder:
- I2Cdev 
- MPU6050
- Rosserial_Arduino_Library



**************************************************************
                      Serial port driver
**************************************************************
The ESP32 has a CH341 usb-to-serial chipset, you can find the Linux Driver in this repository

In case you had a problem while uploading the sketch, related to a missing serial library (happened in Linux), you should install pyserial on your OS:

$pip install pyserial

You may need to add read and write permissions to the new port on Linux , depending on the name of the new port
$chmod a+rw /dev/ttyUSBx 




The MPU9250 9 axis IMU library, is based on:
https://github.com/hideakitai/MPU9250 

The MPU6050 library is based on : 
https://github.com/jrowberg/i2cdevlib/tree/develop/Arduino/MPU6050





