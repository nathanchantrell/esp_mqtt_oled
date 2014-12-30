**esp_mqtt_oled**
=============
This combines Tuan PM's port of the [MQTT client library for ESP8266](https://github.com/tuanpmt/esp_mqtt), ported from: [MQTT client library for Contiki](https://github.com/adamrenner/mqtt-sn-tools-contiki), zarya's [I2C driver](https://github.com/zarya/esp8266_i2c_driver) and the [OLED driver here](http://www.esp8266.com/viewtopic.php?p=4311#p4311) to make a WiFi MQTT display.

It subscribes to three MQTT topics and displays them on the OLED, the display I am using is [this 0.96" 128x64 White OLED](http://www.banggood.com/0_96-Inch-4Pin-White-IIC-I2C-OLED-Display-Module-12864-LED-For-Arduino-p-958196.html) but similar displays are widely available (plenty on eBay).

**Configuration**
I2C address for the OLED is in include/driver/i2c_oled.h
MQTT broker and WiFi settings are in include/user_config.h
GPIO pins to use for I2C are in driver/i2c.h
MQTT topics to subscribe to are in the MQTT_Start() function in user/mqtt.c
What to do with the incoming messages is defined in deliver_publish() in user/mqtt.c

If you want to add more than 3 topics you need to change MQTT_SUB_TOPIC_NUM in user_config.h and the mqtt_topic variable declaration near the top of mqtt.c

I haven't tested the Windows Makefile, it is as it comes with the MQTT demo.
