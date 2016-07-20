#include <Arduino.h>
#include <FS.h>
#include <Ticker.h>
#include <ESP8266wifi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "OneButton.h"
#include "PubSubClient.h"
#include "CmdMessenger.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//IO pin definitions
#define BUTTON_PIN	4
#define RLY_PIN			5

//night light
#define LED_CONTROL_MODE_RESTORE	0
#define LED_CONTROL_MODE_ALS			1
#define LED_CONTROL_MODE_PS				2
#define LED_CONTROL_MODE_ALS_PS		3
#define LED_CONTROL_MODE_BREATHE	4
#define LED_CONTROL_MODE_CMD			5
#define LED_CONTROL_MODE_OFF			6

uint64_t time;
Ticker tick;
void tock();
OneButton button(BUTTON_PIN, true);
void buttonClick();
void buttonHold();

WiFiClient wclient;
PubSubClient mqttClient(wclient);
void startWifiManager(void);
bool loadConfig(void);
bool saveConfig(void);

void mqttSubCallback(const MQTT::Publish& pub);
void mqttController(void);
char mqtt_server[40];
char mqtt_user[33];
char mqtt_token[33];
bool saveSettings;
bool mqtt_connected;

#define BUFFER_SIZE 255
String homeName = "";
String level = "";
String room = "";
String deviceName = "";
String mac = WiFi.macAddress();
String controlTopic = mac;

float altitude;
uint16_t lux;
uint16_t previousLux;
float temperature;
float previousTemperature;
uint16_t humidity;
uint16_t previousHumidity;
float pressure;
float previousPressure;
uint8_t publishAltitude;
bool publishLux;
bool publishTemperature;
bool publishHumidity;
bool publishPressure;
bool publishLsState;

float returnAltitude(float temperature, float pressure, float sealevelPressure);

bool lsisSlave;
String lsMaster="";
bool lsState;
void relayController(void);

uint8_t sensorStatus=0xFF;

CmdMessenger cmdMessenger = CmdMessenger(Serial);
void onReturnStatus(void);
void onReturnDeviceInfo(void);
void onSaveSettings(void);
void onDataSetPushMode(void);
void onDataReturnPushMode(void);
void onDataSetQueryInt(void);
void onDataReturnQueryInt(void);
void onReturnTempQuery(void);
void onReturnHumiQuery(void);
void onReturnPresQuery(void);
void onReturnLuxQuery(void);
void onReturnPSQuery(void);
void onLedSetMode(void);
void onLedReturnMode(void);
void onLedSetFadeLimits(void);
void onLedReturnFadeLimits(void);
void onLedFadeTo(void);
void onCalibratePS(void);
void onSoftReset(void);
enum
{
	kQStatus,							//0
	kRStatus,							//1
	kQDeviceInfo,					//2
	kRDeviceInfo,					//3
	kSSaveSettings,				//4
	kSDataPushMode,				//5
	kQDataPushMode,				//6
	kRDataPushMode,				//7
	kSDataQueryInt,				//8
	kQDataQueryInt,				//9
	kRDataQueryInt,				//10
	kQTemp,								//11
	kRTemp,								//12
	kQHumi,								//13
	kRHumi,								//14
	kQPres,								//15
	kRPres,								//16
	kQLux,								//17
	kRLux,								//18
	kQPS,									//19
	kRPS,									//20
	kSLedMode,						//21
	kQLedMode,						//22
	kRLedMode,						//23
	kSLedModeRestore,			//24
	kSLedFadeMinMax,			//25
	kQLedFadeMinMax,			//26
	kRLedFadeMinMax,			//27
	kSLedCmdFadeTo,				//28
	kSCalibratePS,				//29
	kSReset								//30
};

bool ledState;
void toggleLED2 (void);
void toggleLED2 (void)
{
  ledState^=1;
  digitalWrite(LED_BUILTIN, ledState?HIGH:LOW);
}

void tock()
{
	button.tick();
}

void buttonClick() {
	lsState^=1;
	relayController();
	publishLsState=true;
}

void buttonHold()
{
	WiFiManager wifiManager;
	wifiManager.resetSettings();
	ESP.restart();
}

void relayController()
{
	digitalWrite(RLY_PIN, lsState?HIGH:LOW);
}

void attachCommandCallbacks(void)
{
  cmdMessenger.attach(kRStatus, onReturnStatus);
  cmdMessenger.attach(kRLux, onReturnLuxQuery);
  cmdMessenger.attach(kRTemp, onReturnTempQuery);
  cmdMessenger.attach(kRHumi, onReturnHumiQuery);
  cmdMessenger.attach(kRPres, onReturnPresQuery);
}

void onReturnStatus()
{
	sensorStatus=(uint8_t)cmdMessenger.readInt16Arg();
}

void onReturnLuxQuery()
{
	previousLux = lux;
  lux = cmdMessenger.readInt16Arg();
	if (abs(lux-previousLux)>0) {
		publishLux = true;
	}

	if ((lux<10)&&(publishAltitude==0)) {
		publishAltitude=1;
	}

	if ((lux>30)&&publishAltitude==2) {
		publishAltitude=0;
	}
}

void onReturnTempQuery()
{
	toggleLED2();
	previousTemperature = temperature;
  temperature = (float)cmdMessenger.readInt16Arg();
	temperature = temperature/10.0;
	if (abs(temperature-previousTemperature)>0) {
	  publishTemperature = true;
	}
}

void onReturnHumiQuery()
{
	previousHumidity = humidity;
  humidity = cmdMessenger.readInt16Arg();
	if (abs(humidity-previousHumidity)>1) {
	  publishHumidity = true;
	}
}

void onReturnPresQuery()
{
	previousPressure = pressure;
  pressure = (float)(cmdMessenger.readInt32Arg()/100.0);

	if (abs(pressure-previousPressure)>0.01) {
		publishPressure = true;
	}
}

float returnAltitude(float temperature, float pressure, float sealevelPressure)
{
	//formula from http://keisan.casio.com/exec/system/1224585971
	float altitude = ((pow((sealevelPressure/pressure), (1/5.257))-1)*(temperature+273.15))/0.0065;
	return altitude;
}

void mqttSubCallback(const MQTT::Publish& pub) {
  //get incoming topic
  //strip out useless info
  String incomingCmd = pub.topic().substring(controlTopic.length());
	if (incomingCmd=="/lightSwitch/toggle") {
		buttonClick();
	}

	if (incomingCmd=="/nightLightMode")
	{
		String value = pub.payload_string();
		cmdMessenger.sendCmd(kSLedMode, value.toInt());
	}

	if (incomingCmd=="/dataPushMode") {
		String value = pub.payload_string();
		cmdMessenger.sendCmd(kSDataPushMode, value.toInt());
	}

	if (incomingCmd=="/dataPushInterval") {
		String value = pub.payload_string();
		cmdMessenger.sendCmd(kSDataQueryInt, value.toInt());
	}

	if (incomingCmd=="/deviceName") {
		String value = pub.payload_string();
		deviceName = value;
	}

	if (incomingCmd=="/homeName") {
		String value = pub.payload_string();
		homeName = value;
	}

	if (incomingCmd=="/level") {
		String value = pub.payload_string();
		level = value;
	}

	if (incomingCmd=="/room") {
		String value = pub.payload_string();
		room = value;
	}
}

bool loadConfig()
{

  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/config.json")) {
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_token, json["mqtt_token"]);
					return true;
        }
				else {
					return false;
        }
      }
    }
  }
	else {
		return false;
  }
}

bool saveConfig()
{
		DynamicJsonBuffer jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		json["mqtt_server"] = mqtt_server;
		json["mqtt_user"] = mqtt_user;
		json["mqtt_token"] = mqtt_token;

		File configFile = SPIFFS.open("/config.json", "w");
		if (!configFile) {
			return false;
		}

		json.printTo(configFile);
		configFile.close();
		return true;
}

void saveConfigCallback()
{
		saveSettings=true;
		//cmdMessenger.sendCmd(kSLedMode,LED_CONTROL_MODE_RESTORE);
}

void enterConfigCallback(WiFiManager *myWiFiManager)
{
		cmdMessenger.sendCmd(kSLedMode,LED_CONTROL_MODE_BREATHE);
}

void startWifiManager()
{
	// The extra parameters to be configured (can be either global or just in the setup)
 	// After connecting, parameter.getValue() will get you the configured value
 	// id/name placeholder/prompt default length
	WiFiManagerParameter mqtt_server_entry("server", "mqtt server", mqtt_server, 40);
	WiFiManagerParameter mqtt_user_entry("user", "mqtt user", mqtt_user, 33);
	WiFiManagerParameter mqtt_token_entry("mqtt", "mqtt token", mqtt_token, 33);

	//WiFiManager
	WiFiManager wifiManager;
	wifiManager.setDebugOutput(false);
	//wifiManager.resetSettings();
	wifiManager.setAPCallback(enterConfigCallback);
	wifiManager.setSaveConfigCallback(saveConfigCallback);

	//add all your parameters here
  wifiManager.addParameter(&mqtt_server_entry);
  wifiManager.addParameter(&mqtt_user_entry);
  wifiManager.addParameter(&mqtt_token_entry);
	wifiManager.setTimeout(300);
	wifiManager.setMinimumSignalQuality(14);

	if(!wifiManager.autoConnect()){}
	cmdMessenger.sendCmd(kSLedMode,LED_CONTROL_MODE_RESTORE);
	if (saveSettings == true)
	{
		if (mqtt_server_entry.getValue()!="") {
			//toggleLED2();
			strcpy(mqtt_server, mqtt_server_entry.getValue());
		  strcpy(mqtt_user, mqtt_user_entry.getValue());
		  strcpy(mqtt_token, mqtt_token_entry.getValue());
			saveConfig();
		}
	}
}

void mqttController()
{
	if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {

			if (mqttClient.connect(MQTT::Connect(mac) .set_auth(mqtt_user, mqtt_token) .set_will(controlTopic+"/connected", "", 0, true))) {
        mqttClient.subscribe(MQTT::Subscribe()
													.add_topic(controlTopic+"/lightSwitch/toggle")
                      	);
						mqttClient.publish(MQTT::Publish(controlTopic + "/connected", mac)
															.set_retain(true)
															.set_qos(0)
															);
				}
			}

			if (mqttClient.connected()) {
				if (publishLsState == true)	{
					publishLsState = false;
					mqttClient.publish(MQTT::Publish(controlTopic + "/lightSwitch/state", String(lsState))
						.set_retain(true)
						.set_qos(0)
					);
					delay(10);
				}

		    if (publishLux == true) {
		      publishLux = false;
		      mqttClient.publish(controlTopic + "/sense/lux", String(lux));
					delay(10);
		    }

		    if (publishTemperature == true) {
		      publishTemperature = false;
					mqttClient.publish(controlTopic + "/sense/temperature", String(temperature));
					delay(10);
		    }

		    if (publishHumidity == true) {
		      publishHumidity = false;
		      mqttClient.publish(controlTopic + "/sense/humidity", String(humidity));
					delay(10);
		    }

		    if (publishPressure == true) {
		      publishPressure = false;
		      mqttClient.publish(controlTopic + "/sense/pressure", String(pressure));
					delay(10);
		    }

				if (publishAltitude == 1) {
						publishAltitude = 2;
						altitude = returnAltitude(temperature, pressure, 1013.25);
						mqttClient.publish(MQTT::Publish(controlTopic + "/sense/altitude", String(altitude)) .set_retain(true) .set_qos(0));
						delay(10);
				}
				mqttClient.loop();
				delay(10);
		  }
		}
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(RLY_PIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	tick.attach_ms(1, tock);
	button.setClickTicks(150);
	button.setPressTicks(6000);

  attachCommandCallbacks();
  cmdMessenger.printLfCr();

  Serial.begin(38400);
	//SPIFFS.format();
  delay(5000);
	button.attachClick(buttonClick);
	cmdMessenger.sendCmd(kQStatus);
	if (sensorStatus!=0)
	{
		cmdMessenger.sendCmd(kSReset);
	}

	loadConfig();
	mqttClient.set_server(mqtt_server);
  mqttClient.set_callback(mqttSubCallback);
	startWifiManager();

	button.attachPress(buttonHold);
	ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
	ArduinoOTA.begin();
}

void loop() {
	ArduinoOTA.handle();
  cmdMessenger.feedinSerialData();
	relayController();
	mqttController();
}
