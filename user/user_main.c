#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "os_type.h"
#include "user_config.h"
#include "driver/uart_register.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/i2c_oled.h"

extern void ets_wdt_disable(void);

static volatile bool OLED;

void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

  	i2c_init();
  	OLED = OLED_Init();

	OLED_Print(2, 0, "ESP8266 MQTT OLED", 1);

	wifi_set_opmode(STATION_MODE);
	CFG_Load();
	MQTT_Start();
	INFO("\r\nSystem started ...\r\n");
}
