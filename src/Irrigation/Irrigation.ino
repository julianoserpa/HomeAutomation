#include <Time.h>
#include <TimeAlarms.h>
#include <DS3232RTC.h>
#include <WiFiEsp.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define __TEST_MODE__

#undef _ESPLOGLEVEL_
#define _ESPLOGLEVEL_ 0

#define WIFI_SERIAL_RX_PIN 2
#define WIFI_SERIAL_TX_PIN 3

#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_DEVICE "IrrigationBackyard"
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_SUBSCRIBE_TOPIC "JSC/Home/IrrigationBackyard/Activate"
#define MQTT_PUBLISH_TOPIC "JSC/Home/IrrigationBackyard/SensorsData"

#define DHT_PIN A1
#define DHT_TYPE DHT11

#define LDR_PIN A2

#define RELAY_PIN 4
#define ACTIVATE_BUTTON_PIN 8
#define ACTIVATE_BUZZER_PIN 11
#define RUN_LED_PIN 12

#define ALARM_PUBLISH_SENSORS_DATA_INTERVAL_SECONDS 30

#if defined(__TEST_MODE__)
/**/#define ALARM_ACTIVATE_RELAY_INTERVAL_SECONDS 30
/**/#define ALARM_ACTIVATE_RELAY_DURATION_SECONDS 10
#else
/**/#define ALARM_ACTIVATE_RELAY_HOUR 19
/**/#define ALARM_ACTIVATE_RELAY_DURATION_MINUTES 15
#endif

SoftwareSerial wifiSerial(WIFI_SERIAL_RX_PIN, WIFI_SERIAL_TX_PIN);
WiFiEspClient espClient;
PubSubClient pubSubClient(espClient);
DS3232RTC rtc;
DHT dht(DHT_PIN, DHT_TYPE);

char wifiSSID[] = "Network@Rock";
char wifiPassword[] = "Ntw*rock10";

bool runLedState;
bool relayState;

void setup() {
	initializeSerial();

	initializeWifi();
	initializeMqtt();

	initializePins();

	initializeRtc();
	initializeDHT();

	initializeAlarms();
}

void loop() {
	blinkRunLed();

	printNowDateTime();

	//connectWifi();
	printWifiStatus();

	//connectMqtt();

	checkManualActivation();

	Alarm.delay(1000);
}

void initializeSerial() {
	//Inicializando a serial...
	Serial.begin(9600);

	Serial.println(F("Serial iniciada."));
	Serial.println();
}

void initializeWifi() {
	Serial.println(F("Iniciando Wifi..."));

	wifiSerial.begin(9600);

	WiFi.init(&wifiSerial);

	//Serial.print("Versao do firmware do Wifi ");
	//Serial.print(WiFi.firmwareVersion());
	//Serial.println(".");

	Serial.println(F("Wifi iniciado."));
	Serial.println();
}

void initializeMqtt() {
	Serial.println(F("Iniciando MQTT..."));

	//Serial.print(F("Servidor: "));
	//Serial.print(MQTT_SERVER);
	//Serial.print(':');
	//Serial.println(MQTT_PORT);

	pubSubClient.setServer(MQTT_SERVER, MQTT_PORT);
	pubSubClient.setCallback(mqttCallback);

	pubSubClient.subscribe(MQTT_SUBSCRIBE_TOPIC);

	Serial.println(F("MQTT iniciado."));
	Serial.println();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

}

void initializePins() {
	Serial.println(F("Iniciando pinos..."));

	// Analogs
	Serial.print(F("Pino A"));
	Serial.print(LDR_PIN);
	Serial.println(F(" do LDR, como entrada."));
	pinMode(LDR_PIN, INPUT);

	// Digitals
	Serial.print(F("Pino "));
	Serial.print(RUN_LED_PIN);
	Serial.println(F(" do led de execucao, como saida."));
	pinMode(RUN_LED_PIN, OUTPUT);

	Serial.print(F("Pino "));
	Serial.print(ACTIVATE_BUTTON_PIN);
	Serial.println(F(" do botao de ativacao, como entrada."));
	pinMode(ACTIVATE_BUTTON_PIN, INPUT_PULLUP);

	Serial.print(F("Pino "));
	Serial.print(LED_BUILTIN);
	Serial.println(F(" do led de ativacao, como saida."));
	pinMode(LED_BUILTIN, OUTPUT);

	Serial.print(F("Pino "));
	Serial.print(ACTIVATE_BUZZER_PIN);
	Serial.println(F(" do buzzer de ativacao, como saida."));
	pinMode(ACTIVATE_BUZZER_PIN, OUTPUT);

	Serial.print(F("Pino "));
	Serial.print(RELAY_PIN);
	Serial.println(F(" do rele, como saida."));
	pinMode(RELAY_PIN, OUTPUT);

	Serial.println(F("Pinos iniciados."));
	Serial.println();
}

void initializeRtc() {
	Serial.println(F("Iniciando o RTC..."));

	//TimeElements timeElements;
	//timeElements.Wday = 5; // 1 = Sunday
	//timeElements.Day = 29;
	//timeElements.Month = 1;
	//timeElements.Year = 2017 - 1970; // Offset year used in time library.
	//timeElements.Hour = 23;
	//timeElements.Minute = 16;
	//timeElements.Second = 0;

	//rtc.set(makeTime(timeElements));

	setSyncProvider(rtc.get);

	Serial.println(F("RTC iniciado."));
	Serial.println();
}

void initializeDHT() {
	Serial.println(F("Iniciando o DHT..."));

	dht.begin();

	Serial.println(F("DHT iniciado."));
	Serial.println();
}

void initializeAlarms() {
	Serial.println(F("Iniciando alarmes."));

	Serial.print(F("Publicar dados dos sensores a cada "));
	Serial.print(ALARM_PUBLISH_SENSORS_DATA_INTERVAL_SECONDS);
	Serial.println(F(" segundos."));
	Alarm.timerRepeat(0, 0, ALARM_PUBLISH_SENSORS_DATA_INTERVAL_SECONDS, publishSensorsData);

#if defined(__TEST_MODE__)
	Serial.print(F("Ativar rele a cada "));
	Serial.print(ALARM_ACTIVATE_RELAY_INTERVAL_SECONDS);
	Serial.println(F(" segundos."));
	Alarm.timerRepeat(0, 0, ALARM_ACTIVATE_RELAY_INTERVAL_SECONDS, activateRelay);
#else
	Serial.print(F("Ativar rele as "));
	Serial.print(ALARM_ACTIVATE_RELAY_HOUR);
	Serial.println(F(" horas."));
	Alarm.alarmRepeat(ALARM_ACTIVATE_RELAY_HOUR, 0, 0, activateRelay);
#endif

	Serial.println(F("Alarmes iniciados."));
	Serial.println();
}

void publishSensorsData() {
	if (!pubSubClient.connected())
		return;

	float humidity = dht.readHumidity();
	float temperature = dht.readTemperature();
	float heatIndex = dht.computeHeatIndex(temperature, humidity, false);

	int ldrReadedValue = analogRead(LDR_PIN);
	long luminosity = map(ldrReadedValue, 0, 1023, 0, 255);

	StaticJsonBuffer<200> jsonBuffer;

	JsonObject& jsonPublishData = jsonBuffer.createObject();
	jsonPublishData["thing"] = "IrrigationBackyard";
	jsonPublishData["dateTime"] = now();

	JsonArray& jsonSensorsData = jsonPublishData.createNestedArray("sensorsData");

	JsonObject& jsonDhtData = jsonSensorsData.createNestedObject();
	jsonDhtData["sensorType"] = "dht";
	jsonDhtData["humidity"] = humidity;
	jsonDhtData["temperature"] = temperature;
	jsonDhtData["heatIndex"] = heatIndex;

	JsonObject& jsonLdrData = jsonSensorsData.createNestedObject();
	jsonLdrData["sensorType"] = "ldr";
	jsonLdrData["luminosity"] = luminosity;

	char publishDataBuffer[200];

	jsonPublishData.printTo(publishDataBuffer, sizeof(publishDataBuffer));
	jsonPublishData.prettyPrintTo(Serial);

	pubSubClient.publish(MQTT_PUBLISH_TOPIC, publishDataBuffer);
}

void activateRelay() {
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(RELAY_PIN, HIGH);

	tone(ACTIVATE_BUZZER_PIN, 500, 100);

	relayState = true;

#if defined(__TEST_MODE__)
	int alarmActivateRelayDurationSeconds = ALARM_ACTIVATE_RELAY_DURATION_SECONDS;
#else
	int alarmActivateRelayDurationSeconds = ALARM_ACTIVATE_RELAY_DURATION_MINUTES * 60;
#endif

	Serial.print(F("Rele ativado por "));
	Serial.print(alarmActivateRelayDurationSeconds);
	Serial.println(F(" segundos."));
	Serial.println();

	Alarm.timerOnce(alarmActivateRelayDurationSeconds, deactivateRelay);
}

void deactivateRelay() {
	digitalWrite(RELAY_PIN, LOW);
	digitalWrite(LED_BUILTIN, LOW);

	relayState = false;

	Serial.println(F("Rele desativado."));
	Serial.println();
}

void blinkRunLed() {
	if (runLedState)
		digitalWrite(RUN_LED_PIN, LOW);
	else
		digitalWrite(RUN_LED_PIN, HIGH);

	runLedState = !runLedState;
}

void printNowDateTime() {
	time_t nowTime = now();

	Serial.print(F("Data/Hora: "));
	Serial.print(day(nowTime));
	Serial.print('/');
	Serial.print(month(nowTime));
	Serial.print('/');
	Serial.print(year(nowTime));
	Serial.print(' ');
	Serial.print(hour(nowTime));
	Serial.print(':');
	Serial.print(minute(nowTime));
	Serial.print(':');
	Serial.println(second(nowTime));
	Serial.println();
}

void connectWifi() {
	//if (wifiStatus != WL_CONNECTED) {
		//Serial.print("Conectando no Wifi ");
		//Serial.print(wifiSSID);
		//Serial.print("...");

	int wifiStatus = WL_IDLE_STATUS;
	short attemptCount = 5;

	while (wifiStatus != WL_CONNECTED && attemptCount > 0) {
		attemptCount--;

		wifiStatus = WiFi.begin(wifiSSID, wifiPassword);

		//Serial.print('.');
		Alarm.delay(1000);
	}
	//}

	//Serial.print(F("Conectado no Wifi com sucesso."));
	Serial.println();
}

void printWifiStatus() {
	Serial.print(F("Wifi status: "));
	Serial.println(WiFi.status());

	Serial.print(F("Nome da Rede: "));
	Serial.println(WiFi.SSID());

	Serial.print(F("Mascara da Rede: "));
	Serial.println(WiFi.subnetMask());

	Serial.print(F("IP da Rede: "));
	Serial.println(WiFi.localIP());

	Serial.print(F("IP do Gateway: "));
	Serial.println(WiFi.gatewayIP());

	byte macAddress[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(macAddress);

	char macAddressBuffer[18];
	sprintf(macAddressBuffer,
		"%02X:%02X:%02X:%02X:%02X:%02X",
		macAddress[5], macAddress[4], macAddress[3], macAddress[2], macAddress[1], macAddress[0]);

	Serial.print(F("Endereco do MAC: "));
	Serial.println(macAddressBuffer);

	Serial.print(F("Forca do sinal: "));
	Serial.print(WiFi.RSSI());
	Serial.println(F("dBm"));

	Serial.println();
}

void connectMqtt() {
	//if (!pubSubClient.connected()) {
		//Serial.print(F("Conectando no MQTT "));
		//Serial.print(MQTT_SERVER);
		//Serial.print(':');
		//Serial.print(MQTT_PORT);
		//Serial.print(F(" com o usuario "));
		//Serial.print(MQTT_USER);
		//Serial.print(F(" e dispositivo "));
		//Serial.print(MQTT_DEVICE);
		//Serial.print(F("..."));

	short attemptCount = 5;

	while (!pubSubClient.connected() && attemptCount > 0) {
		attemptCount--;

		//pubSubClient.connect(MQTT_DEVICE, MQTT_USER, MQTT_PASSWORD);
		pubSubClient.connect(MQTT_DEVICE);

		//Serial.print('.');
		Alarm.delay(1000);
	}
	//}

	pubSubClient.loop();

	//Serial.println(F("Conectado no MQTT com sucesso."));
	Serial.println();
}

//void printDhtData() {
//	float humidity = dht.readHumidity();
//	float temperature = dht.readTemperature();
//	float heatIndex = dht.computeHeatIndex(temperature, humidity, false);
//
//	Serial.print(F("Umidade: "));
//	Serial.print(humidity);
//	Serial.println('%');
//
//	Serial.print(F("Temperatura: "));
//	Serial.print(temperature);
//	Serial.println(F("*C"));
//
//	Serial.print(F("Sensacao Termica: "));
//	Serial.print(heatIndex);
//	Serial.println(F("*C"));
//}

//void printLdrData() {
//	int readedValue = analogRead(LDR_PIN);
//
//	float luminosity = map(readedValue, 0, 1023, 0, 255);
//
//	Serial.print(F("Luminosidade: "));
//	Serial.print(luminosity);
//	Serial.println(F("Lux"));
//}

void checkManualActivation() {
	int buttonValue = digitalRead(ACTIVATE_BUTTON_PIN);

	if (buttonValue == LOW) {
		if (relayState) {
			Serial.println(F("Disparada ativacao manual."));
			deactivateRelay();
		}
		else {
			Serial.println(F("Disparada desativacao manual."));
			activateRelay();
		}

		Serial.println();
	}
}
