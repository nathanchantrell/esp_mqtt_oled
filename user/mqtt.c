#include "user_interface.h"
#include "osapi.h"
#include "espconn.h"
#include "os_type.h"
#include "mem.h"
#include "mqtt_msg.h"
#include "debug.h"
#include "user_config.h"
#include "config.h"
#include "driver/uart.h"

struct espconn *pMqttConn;
os_timer_t japDelayChack;
BOOL wifiReady = FALSE;
#define mqtt_procTaskPrio       0
#define mqtt_procTaskQueueLen    1
os_event_t    mqtt_procTaskQueue[mqtt_procTaskQueueLen];

static ETSTimer WiFiLinker, mqttKeeper;


struct espconn *pCon;
static ip_addr_t mqtt_ip;



uint8_t client_id[32];
static char mqtt_topic[3][64];

uint8_t mqtt_user[] = MQTT_USER;
uint8_t mqtt_pass[] = MQTT_PASS;

uint16_t keepAlive = 120;
uint32_t keepAliveTick = 0;
uint32_t reconnectCounter = 0;
uint32_t mqttConnected = 0, mqttConnectCouter = 0;

static char pub_topic[32];
static uint8_t topic_idx = 0;

char* infomsg = "";
char* ext_temp = "";
char* room_temp = "";

typedef struct mqtt_event_data_t
{
  uint8_t type;
  const char* topic;
  const char* data;
  uint16_t topic_length;
  uint16_t data_length;
  uint16_t data_offset;

} mqtt_event_data_t;

typedef struct mqtt_state_t
{
  uint16_t port;
  int auto_reconnect;
  mqtt_connect_info_t* connect_info;

  uint8_t* in_buffer;
  uint8_t* out_buffer;
  int in_buffer_length;
  int out_buffer_length;
  uint16_t message_length;
  uint16_t message_length_read;
  mqtt_message_t* outbound_message;
  mqtt_connection_t mqtt_connection;
  uint16_t pending_msg_id;
  int pending_msg_type;

} mqtt_state_t;

static mqtt_connect_info_t connect_info =
{
    .client_id = client_id,
    .username = mqtt_user,
    .password = mqtt_pass,
    .will_topic = NULL,
    .will_message = NULL,
    .keepalive = 30,
    .will_qos = 0,
    .will_retain = 0,
    .clean_session = 1
};

typedef enum {
	WIFI_INIT,
	WIFI_CONNECTING,
	WIFI_CONNECTING_ERROR,
	WIFI_CONNECTED,
	DNS_RESOLVE,
	TCP_DISCONNECTED,
	TCP_RECONNECT_REQ,
	TCP_RECONNECT,
	TCP_CONNECTING,
	TCP_CONNECTING_ERROR,
	TCP_CONNECTED,
	MQTT_CONNECT_SEND,
	MQTT_CONNECT_SENDING,
	MQTT_SUBSCIBE_SEND,
	MQTT_SUBSCIBE_SENDING,
	MQTT_DATA,
	MQTT_PUBLISH_RECV,
	MQTT_PUBLISHING
} tConnState;

LOCAL uint8_t mqttTxBuf[1024] __attribute__((aligned(4))); // must big enough to hold a packet of all types
LOCAL uint8_t mqttRxBuf[1024] __attribute__((aligned(4)));

mqtt_state_t mqtt_state;

tConnState connState = WIFI_INIT;
/**
  * @brief  Task of process command or txdata.
  * @param  events: no used
  * @retval None
  */


static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg)
{
	struct ip_info ipConfig;
	struct station_config stationConf;

	os_timer_disarm(&WiFiLinker);
	wifi_get_ip_info(STATION_IF, &ipConfig);
	if (wifi_station_get_connect_status() == STATION_GOT_IP && ipConfig.ip.addr != 0)
	{
		if(connState < WIFI_CONNECTED){
			connState = WIFI_CONNECTED;

			INFO("Wifi connected\r\n");
		} else if(connState == WIFI_CONNECTED && mqttConnected == 0){
			if(mqttConnectCouter < MQTT_CONNTECT_TIMER){
				mqttConnectCouter ++;
			} else {
				mqttConnectCouter = 0;
				connState = WIFI_CONNECTED;
			}
		}
		if(connState == TCP_RECONNECT_REQ){
			connState = WIFI_CONNECTED;
		}
		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 5000, 0);

	}
	else
	{
		if(wifi_station_get_connect_status() == STATION_WRONG_PASSWORD)
		{
			connState = WIFI_CONNECTING_ERROR;
			INFO("STATION_WRONG_PASSWORD\r\n");
			wifi_station_connect();

		}
		else if(wifi_station_get_connect_status() == STATION_NO_AP_FOUND)
		{
			connState = WIFI_CONNECTING_ERROR;
			INFO("STATION_NO_AP_FOUND\r\n");
			wifi_station_connect();

		}
		else if(wifi_station_get_connect_status() == STATION_CONNECT_FAIL)
		{
			connState = WIFI_CONNECTING_ERROR;
			INFO("STATION_CONNECT_FAIL\r\n");
			wifi_station_connect();
		}
		else
		{
			//connState = WIFI_CONNECTING;
			INFO("STATION_IDLE\r\n");
		}
		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 1000, 0);
	}
	system_os_post(mqtt_procTaskPrio, 0, 0);
}

LOCAL void ICACHE_FLASH_ATTR
mqtt_dns_found(const char *name, ip_addr_t *ipaddr, void *arg)
{

	INFO("DNS FOUND\r\n");
  if(ipaddr == NULL)
  {
// 		device_status = DEVICE_CONNECT_SERVER_FAIL;
	  INFO("BUT GOT NO IP\r\n");
    return;
  }

  os_printf("DNS found: %d.%d.%d.%d\n",
            *((uint8 *) &ipaddr->addr),
            *((uint8 *) &ipaddr->addr + 1),
            *((uint8 *) &ipaddr->addr + 2),
            *((uint8 *) &ipaddr->addr + 3));

  if(mqtt_ip.addr == 0 && ipaddr->addr != 0)
  {
	 os_memcpy(pCon->proto.tcp->remote_ip, &ipaddr->addr, 4);
	 espconn_connect(pCon);
	 connState = TCP_CONNECTING;
	 INFO("TCP_CONNECTING\r\n");
  }
  system_os_post(mqtt_procTaskPrio, 0, 0);
}


int mqtt_publish_with_length(const char* topic, const char* data, int data_length, int qos, int retain)
{

  INFO("MQTT: sending publish...\r\n");
  mqtt_state.outbound_message = mqtt_msg_publish(&mqtt_state.mqtt_connection,
                                                 topic, data, data_length,
                                                 qos, retain,
                                                 &mqtt_state.pending_msg_id);
  return 0;
}

static void deliver_publish(mqtt_state_t* state, uint8_t* message, int length)
{
	uint8_t buff[100];
	uint8_t topic[100];

	mqtt_event_data_t event_data;

	event_data.topic_length = length;
	event_data.topic = mqtt_get_publish_topic(message, &event_data.topic_length);

	event_data.data_length = length;
	event_data.data = mqtt_get_publish_data(message, &event_data.data_length);

	os_memset(topic, 0, sizeof(topic));
	os_memcpy(topic, (char*)event_data.topic, event_data.topic_length);
	os_strcpy(sysCfg.ota_host, topic);

	os_memset(buff, 0, sizeof(buff));
	os_memcpy(buff, (char*)event_data.data, event_data.data_length);
	os_strcpy(sysCfg.ota_host, buff);

	INFO("New message on topic: ");
	INFO(topic);	
	INFO("\r\n");
	INFO("Message: ");
	INFO(buff);
	INFO("\r\n");

	if ((strcmp(topic,(uint8_t*)"external/temperature/rear"))==0) {
		strcat(buff, "C");
		strcpy(ext_temp,buff);
	        OLED_Print(1, 0, "External", 1);
		OLED_Print(1, 2, "       ", 2);
		OLED_Print(1, 2, ext_temp, 2);
	} else if ((strcmp(topic,(uint8_t*)"desk/temperature"))==0) {
		strcat(buff, "C");
		strcpy(ext_temp,buff);
	        OLED_Print(14, 0, "Room", 1);
		OLED_Print(9, 2, "       ", 2);
		OLED_Print(9, 2, room_temp, 2);
	} else if ((strcmp(topic,(uint8_t*)"displays/esp8266oled2"))==0) {
		strcpy(infomsg,buff);
		OLED_Print(0, 7, "                    ", 1);
		OLED_Print(0, 7, infomsg, 1);
	}

}



/**
  * @brief  Client received callback function.
  * @param  arg: contain the ip link information
  * @param  pdata: received data
  * @param  len: the lenght of received data
  * @retval None
  */
void ICACHE_FLASH_ATTR
mqtt_tcpclient_recv(void *arg, char *pdata, unsigned short len)
{
	uint8_t msg_type;
	uint8_t msg_qos;

	uint16_t msg_id;

	INFO("RECEIVED\r\n");
	if(len < MQTT_BUF_SIZE && len > 0){
		memcpy(mqtt_state.in_buffer, pdata, len);

		switch(connState){
		case MQTT_CONNECT_SENDING:
			if(mqtt_get_type(mqtt_state.in_buffer) != MQTT_MSG_TYPE_CONNACK){
				INFO("MQTT: Invalid packet\r\n");
				espconn_disconnect(pCon);
			} else {
				INFO("MQTT: Connected\r\n");
				connState = MQTT_SUBSCIBE_SEND;
				OLED_CLS();
				OLED_Print(4, 7, "MQTT Connected", 1);
			}
			break;
		case MQTT_SUBSCIBE_SENDING:
		//	if(mqtt_get_type(mqtt_state.in_buffer) != MQTT_MSG_TYPE_SUBACK){
		//		INFO("MQTT: Invalid packet\r\n");
		//		espconn_disconnect(pCon);
		//	} else {
				INFO("MQTT: Subscribe successful\r\n");
				topic_idx ++;
				if(topic_idx >= MQTT_SUB_TOPIC_NUM){
					connState = MQTT_DATA;
					topic_idx = 0;
				} else {
					connState = MQTT_SUBSCIBE_SEND;
				}

		//	}
			break;
		case MQTT_DATA:
			mqtt_state.message_length_read = len;
			mqtt_state.message_length = mqtt_get_total_length(mqtt_state.in_buffer, mqtt_state.message_length_read);
			msg_type = mqtt_get_type(mqtt_state.in_buffer);
			msg_qos = mqtt_get_qos(mqtt_state.in_buffer);
			msg_id = mqtt_get_id(mqtt_state.in_buffer, mqtt_state.in_buffer_length);
			switch(msg_type)
			{
			  case MQTT_MSG_TYPE_SUBACK:
				if(mqtt_state.pending_msg_type == MQTT_MSG_TYPE_SUBSCRIBE && mqtt_state.pending_msg_id == msg_id)
				  INFO("MQTT: Subscribe successful\r\n");
				break;
			  case MQTT_MSG_TYPE_UNSUBACK:
				if(mqtt_state.pending_msg_type == MQTT_MSG_TYPE_UNSUBSCRIBE && mqtt_state.pending_msg_id == msg_id)
				  INFO("MQTT: UnSubscribe successful\r\n");
				break;
			  case MQTT_MSG_TYPE_PUBLISH:
				if(msg_qos == 1)
				  mqtt_state.outbound_message = mqtt_msg_puback(&mqtt_state.mqtt_connection, msg_id);
				else if(msg_qos == 2)
				  mqtt_state.outbound_message = mqtt_msg_pubrec(&mqtt_state.mqtt_connection, msg_id);

				deliver_publish(&mqtt_state, mqtt_state.in_buffer, mqtt_state.message_length_read);
				break;
			  case MQTT_MSG_TYPE_PUBACK:
				if(mqtt_state.pending_msg_type == MQTT_MSG_TYPE_PUBLISH && mqtt_state.pending_msg_id == msg_id){
				  INFO("MQTT: Publish successful\r\n");

				}

				break;
			  case MQTT_MSG_TYPE_PUBREC:
				mqtt_state.outbound_message = mqtt_msg_pubrel(&mqtt_state.mqtt_connection, msg_id);
				break;
			  case MQTT_MSG_TYPE_PUBREL:
				mqtt_state.outbound_message = mqtt_msg_pubcomp(&mqtt_state.mqtt_connection, msg_id);
				break;
			  case MQTT_MSG_TYPE_PUBCOMP:
				if(mqtt_state.pending_msg_type == MQTT_MSG_TYPE_PUBLISH && mqtt_state.pending_msg_id == msg_id){
				  INFO("MQTT: Public successful\r\n");

				}
				break;
			  case MQTT_MSG_TYPE_PINGREQ:
				mqtt_state.outbound_message = mqtt_msg_pingresp(&mqtt_state.mqtt_connection);
				break;
			  case MQTT_MSG_TYPE_PINGRESP:
				// Ignore
				break;
			}
			// NOTE: this is done down here and not in the switch case above
			// because the PSOCK_READBUF_LEN() won't work inside a switch
			// statement due to the way protothreads resume.
			if(msg_type == MQTT_MSG_TYPE_PUBLISH)
			{
			  uint16_t len;

			  // adjust message_length and message_length_read so that
			  // they only account for the publish data and not the rest of the
			  // message, this is done so that the offset passed with the
			  // continuation event is the offset within the publish data and
			  // not the offset within the message as a whole.
			  len = mqtt_state.message_length_read;
			  mqtt_get_publish_data(mqtt_state.in_buffer, &len);
			  len = mqtt_state.message_length_read - len;
			  mqtt_state.message_length -= len;
			  mqtt_state.message_length_read -= len;

			  if(mqtt_state.message_length_read < mqtt_state.message_length)
			  {
				  msg_type = MQTT_PUBLISH_RECV;
			  }

			}
			break;
			case MQTT_PUBLISH_RECV:
							//mqtt_state.message_length_read += uip_len;
							//PSOCK_READBUF_LEN(&state->ps, state->message_length - state->message_length_read);
								//deliver_publish_continuation(&mqtt_state, mqtt_state.message_length_read, mqtt_state.in_buffer, PSOCK_DATALEN(&state->ps));
								//state->message_length_read += PSOCK_DATALEN(&state->ps);

				break;
		}
	}
	system_os_post(mqtt_procTaskPrio, 0, 0);
}

/**
  * @brief  Client send over callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
void ICACHE_FLASH_ATTR
mqtt_tcpclient_sent_cb(void *arg)
{
	INFO("SENT\r\n");
	system_os_post(mqtt_procTaskPrio, 0, 0);
}


void ICACHE_FLASH_ATTR
mqtt_tcpclient_discon_cb(void *arg) {

	os_free(pCon->proto.tcp);
	os_free(pCon);
	mqttConnected = 0;
	INFO("DISCONNECT CALLBACK\r\n");
	connState = WIFI_CONNECTED;
	system_os_post(mqtt_procTaskPrio, 0, 0);
}

/**
  * @brief  Tcp client connect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
void ICACHE_FLASH_ATTR
mqtt_tcpclient_connect_cb(void *arg)
{
  espconn_regist_disconcb(pCon, mqtt_tcpclient_discon_cb);
  espconn_regist_recvcb(pCon, mqtt_tcpclient_recv);////////
  espconn_regist_sentcb(pCon, mqtt_tcpclient_sent_cb);///////
  INFO("CONNECT CALLBACK\r\n");
  connState = MQTT_CONNECT_SEND;
  mqttConnected = 1;
  system_os_post(mqtt_procTaskPrio, 0, 0);
}

/**
  * @brief  Tcp client connect repeat callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
void ICACHE_FLASH_ATTR
mqtt_tcpclient_recon_cb(void *arg, sint8 errType)
{
	INFO("RECONNECT CALLBACK\r\n");
	os_free(pCon->proto.tcp);
	os_free(pCon);
	connState = TCP_RECONNECT_REQ;
	mqttConnected = 0;
	reconnectCounter ++;
	if(reconnectCounter > MQTT_RECONNECT_TIMEOUT){
		reconnectCounter = 0;
		connState = WIFI_INIT;
	}
	system_os_post(mqtt_procTaskPrio, 0, 0);

}


void ICACHE_FLASH_ATTR mqtt_keepalive(void *arg)
{
	if(connState == MQTT_DATA){
		keepAliveTick ++;
		if(keepAliveTick > mqtt_state.connect_info->keepalive){

			INFO("\r\nMQTT: Send keepalive packet!\r\n");
			mqtt_state.outbound_message = mqtt_msg_pingreq(&mqtt_state.mqtt_connection);
			keepAliveTick = 0;

		}
		system_os_post(mqtt_procTaskPrio, 0, 0);
	}
	os_timer_disarm(&mqttKeeper);
	os_timer_arm(&mqttKeeper, 1000, 0);
}
void ICACHE_FLASH_ATTR
MQTT_Sub()
{
	INFO("MQTT: Send subscribe, topic:");
	INFO(mqtt_topic[topic_idx]);
	INFO("\r\n");
	mqtt_state.outbound_message = mqtt_msg_subscribe(&mqtt_state.mqtt_connection,
										   mqtt_topic[topic_idx], 0,
										   &mqtt_state.pending_msg_id);
	mqtt_state.pending_msg_type = MQTT_MSG_TYPE_SUBSCRIBE;
	espconn_sent(pCon, mqtt_state.outbound_message->data, mqtt_state.outbound_message->length);
	mqtt_state.outbound_message = NULL;
	connState = MQTT_SUBSCIBE_SENDING;
}

void ICACHE_FLASH_ATTR
MQTT_Task(os_event_t *events)
{
	struct station_config stationConf;
	INFO(".");

	switch(connState){
	case WIFI_INIT:
		INFO("WIFI_INIT\r\n");
		os_memset(&stationConf, 0, sizeof(struct station_config));
		os_sprintf(stationConf.ssid, "%s", sysCfg.sta_ssid);
		os_sprintf(stationConf.password, "%s", sysCfg.sta_pwd);

		wifi_station_set_config(&stationConf);
		os_timer_disarm(&WiFiLinker);
		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 1000, 0);

		os_timer_disarm(&mqttKeeper);
		os_timer_setfn(&mqttKeeper, (os_timer_func_t *)mqtt_keepalive, NULL);
		os_timer_arm(&mqttKeeper, 1000, 0);

		wifi_station_set_auto_connect(TRUE);
		wifi_station_connect();
		connState = WIFI_CONNECTING;
		break;
	case WIFI_CONNECTED:

		OLED_Print(4, 4, "WiFi Connected", 1);

		pCon = (struct espconn *)os_zalloc(sizeof(struct espconn));
		pCon->type = ESPCONN_TCP;
		pCon->state = ESPCONN_NONE;
		pCon->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
		pCon->proto.tcp->local_port = espconn_port();
		pCon->proto.tcp->remote_port = sysCfg.mqtt_port;
		espconn_regist_connectcb(pCon, mqtt_tcpclient_connect_cb);
		espconn_regist_reconcb(pCon, mqtt_tcpclient_recon_cb);

		if(UTILS_StrToIP(sysCfg.mqtt_host, &pCon->proto.tcp->remote_ip)) {
			INFO("CONNECT TO IP:");
			INFO(sysCfg.mqtt_host);
			INFO("\r\n");
			espconn_connect(pCon);
			connState = TCP_CONNECTING;
		}
		else {
			INFO("CONNECT TO HOST:");
			INFO(sysCfg.mqtt_host);
			INFO("\r\n");
			espconn_gethostbyname(pCon, sysCfg.mqtt_host, &mqtt_ip, mqtt_dns_found);
			connState = DNS_RESOLVE;
		}
		break;
	case TCP_RECONNECT:
		espconn_connect(pCon);
		connState = TCP_CONNECTING;
		break;
	case MQTT_CONNECT_SEND:
		connect_info.keepalive = keepAlive;
		mqtt_msg_init(&mqtt_state.mqtt_connection, mqtt_state.out_buffer, mqtt_state.out_buffer_length);
		mqtt_state.outbound_message = mqtt_msg_connect(&mqtt_state.mqtt_connection, mqtt_state.connect_info);
		espconn_sent(pCon, mqtt_state.outbound_message->data, mqtt_state.outbound_message->length);
		//uip_send(mqtt_state.outbound_message->data, mqtt_state.outbound_message->length);
		connState = MQTT_CONNECT_SENDING;
		mqtt_state.outbound_message = NULL;
		break;
	case MQTT_SUBSCIBE_SEND:
		MQTT_Sub();
		break;
	case MQTT_DATA:
		if(mqtt_state.outbound_message != NULL){
			espconn_sent(pCon, mqtt_state.outbound_message->data, mqtt_state.outbound_message->length);
			mqtt_state.outbound_message = NULL;
			if(mqtt_state.pending_msg_type == MQTT_MSG_TYPE_PUBLISH && mqtt_state.pending_msg_id == 0)
				INFO("MQTT: Publish message is done!\r\n");
			break;
		}
		break;
	}
}


void MQTT_Start()
{
	connState = WIFI_INIT;
	os_sprintf(client_id, MQTT_CLIENT_ID, sysCfg.device_id);	//MQTT client id

	os_sprintf(mqtt_topic[0], "external/temperature/rear",sysCfg.device_id);  // 1st topic to subscribe to

	os_sprintf(mqtt_topic[1], "desk/temperature", sysCfg.device_id); 	// 2nd topic to subscribe to

	os_sprintf(mqtt_topic[2], "displays/esp8266oled2", sysCfg.device_id); 	// 3rd topic to subscribe to

	os_sprintf(pub_topic, "/%08X/send", sysCfg.device_id);		// send data to topic: /chipid/send

    mqtt_state.in_buffer = mqttRxBuf;
    mqtt_state.in_buffer_length =sizeof(mqttRxBuf);
    mqtt_state.out_buffer =  mqttTxBuf;
    mqtt_state.out_buffer_length = sizeof(mqttTxBuf);
    mqtt_state.connect_info = &connect_info;

	system_os_task(MQTT_Task, mqtt_procTaskPrio, mqtt_procTaskQueue, mqtt_procTaskQueueLen);
	system_os_post(mqtt_procTaskPrio, 0, 0);
}

void MQTT_Pub(uint32_t data)
{
	char buf[64];
	if(connState != MQTT_DATA)
		return;

	uint16_t len = os_sprintf(buf, "{\"type\":\"pir\", \"data\":\"%d\"}", data);
	mqtt_publish_with_length(pub_topic, buf, len, 0, 0);
}



