# ESP8266_Fan_Control

I built a NAS expansion chassis for my freenas server. This new chassis is only 8 hard-drives, a PSU with a dual PSU adapter and a Supermicro AOM-SAS3-8I8E (SAS 12Gb). 
3 fans were powered by the PSU and were loud so I changed them to install 3 Noctua 12V 80mm PWM fans. 

ESP8266 that runs a PID controller. It reads two DS18B20 and controls 3 noctua fan (also checks RPM feedback) and display everything on a webserver. Pretty simple.


Electronics drawing:

![ESP8266](/README/ESP8266.JPG)

Web Interface:

![Web interface](/README/WebServer.JPG)
