#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPUpdateServer.h>
#include <SoftWire.h>
#include <I2CSoftSlave.h>
#include <PubSubClient.h>
#include <AsyncDelay.h>

static AsyncDelay timeout;


#define MQTT_VERSION MQTT_VERSION_3_1_1

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "bedroom_light1";
const PROGMEM char* MQTT_SERVER_IP = "192.168.1.24";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "sonoff";
const PROGMEM char* MQTT_PASSWORD = "testpassword";

// MQTT: topics
const char* MQTT_LIGHT_STATE_TOPIC = "bedroom/light1/status";
const char* MQTT_LIGHT_COMMAND_TOPIC = "bedroom/light1/switch";

// payloads by default (on/off)
const char* LIGHT_ON = "ON";
const char* LIGHT_OFF = "OFF";

// brightness
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC = "bedroom/light1/brightness/status";
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC = "bedroom/light1/brightness/set";

WiFiClient wifiClient;
PubSubClient client(wifiClient);


const char* ssid = "NET";
const char* password = "password";
const char *software_version = "3.1";

const uint8_t sdaPin = 5;
const uint8_t sclPin = 4;
SoftWire i2c(sdaPin, sclPin);
int ota_running = 0;

uint8_t light = -1;
volatile int set_light = -1;
int destination_light = -1;

void setLight(byte x);

// buffer used to send/receive data with MQTT
const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE];



ESP8266WebServer server ( 80 );
ESP8266HTTPUpdateServer httpUpdater;


        // Count up package
volatile int packet_count = 0;
i2c_soft_slave::i2c_packet packets[100];

volatile byte i2c_write_reg = 0;
void i2c_read(i2c_soft_slave::i2c_packet *pack) {
  if (pack->addr == 0x51 && i2c_write_reg == 0x05) {
    pack->data[0] = light;
    i2c_write_reg = 0;
  }
}
void i2c_stop(i2c_soft_slave::i2c_packet *pack) {
  // Example, set register 0x02 addr: 0x51 data: 0x00, 0x02
  if (pack->w) {
    if (pack->addr == 0x51 && pack->dc == 2 && pack->data[0] == 0)
      i2c_write_reg = pack->data[1];

    // After selecting register, we can fetch value
    else if (pack->addr == 0x51 && pack->dc == 1 && i2c_write_reg == 0x02) {
      set_light = pack->data[0];
      i2c_write_reg = 0;
    }
  }

  // Save for later logging.
  //memcpy(&packets[packet_count++ % 100], pack, sizeof(i2c_soft_slave::i2c_packet));
}

i2c_soft_slave i2c_slave(12, 13, &i2c_read, &i2c_stop);

// function called to publish the state of the light (on/off)
void publishLightState() {
  if (light > 1) {
    printf("publish %s %s\n", MQTT_LIGHT_STATE_TOPIC, LIGHT_ON);
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_ON, true);
  } else {
    printf("publish %s %s\n", MQTT_LIGHT_STATE_TOPIC, LIGHT_OFF);
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
  }
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", light);
  printf("publish %s %s\n", MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer);
  client.publish(MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  printf("Callback '%s' '%s'\n", p_topic, payload.c_str());

  // handle message topic
  if (String(MQTT_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON)) && light <= 1 && set_light < 1 && destination_light < 1) {
      set_light = 19;
    } else if (payload.equals(String(LIGHT_OFF)) && (light > 0 || set_light > 0 || destination_light > 0)) {
      set_light = 0;
    }
  } else if (String(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    uint8_t brightness = payload.toInt();
    printf("Brightness %d\n", brightness);
    set_light = brightness+1;
  }
  publishLightState();

}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting New 3.0");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  printf( "Connected to: %s\n", ssid);
  printf( "IP address: %s\n", WiFi.localIP().toString().c_str());
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u bytes\n\n", realSize);

  Serial.printf("Flash ide  size: %u bytes\n", ideSize);
  Serial.printf("Flash ide speed: %u Hz\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    Serial.println("Flash Chip configuration wrong!\n");
  } else {
    Serial.println("Flash Chip configuration ok.\n");
  }

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("esp8266-wakeup");
  ArduinoOTA.onStart([]() {
    printf("Start\n");
    ota_running = 1;
  });
  ArduinoOTA.onEnd([]() {
    printf("\nEnd\n");
    ota_running = 0;
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int old_percent = 0;
    int percent = progress / (total / 100);
    if (percent != old_percent) {
      printf("Progress: %u%%\r", percent);
      old_percent = percent;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: \n", error);
    if (error == OTA_AUTH_ERROR) Serial.printf("Auth Failed\n");
    else if (error == OTA_BEGIN_ERROR) Serial.printf("Begin Failed\n");
    else if (error == OTA_CONNECT_ERROR) Serial.printf("Connect Failed\n");
    else if (error == OTA_RECEIVE_ERROR) Serial.printf("Receive Failed\n");
    else if (error == OTA_END_ERROR) Serial.printf("End Failed\n");
  });
  ArduinoOTA.begin();


  if ( MDNS.begin( "esp8266-wakeup" ) ) {
    printf( "MDNS responder started\n" );
  }


  printf("Setup i2c\n");
  i2c.setDelay_us(40);
  i2c.setTimeout_ms(40);
  i2c.begin();

  printf("Register handlers\n");

  server.on ( "/", []() {
    if (server.hasArg("val")) {
      String val = server.arg("val");
      int ival = val.toInt();
      setLight(ival);
    }
    char buf[1024];
    size_t pos = snprintf(buf, 1024, "Version: %s Light value: %d<br>", software_version, light);
    for (size_t i = 0; i <= 20; i++)
      if (light == i)
        pos += snprintf(buf + pos, 1024 - pos, "<b><a href=\"/?val=%d\">%d</a></b> ", i, i);
      else
        pos += snprintf(buf + pos, 1024 - pos, "<a href=\"/?val=%d\">%d</a> ", i, i);

    server.send ( 200, "text/html", buf);
  });
  httpUpdater.setup(&server);
  server.begin();

        pinMode(sdaPin, OUTPUT);
      pinMode(sclPin, OUTPUT);

  printf("MQTT\n");
  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);

  timeout.start(5000, AsyncDelay::MILLIS);
  printf("Ready\n");
}

/*
  char hex_value[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

  int hex2int(char hex) {
   for (int i=0; i < 16; i++) {
      if (hex_value[i] == toupper(hex)) {
         return i;
      }
   }
   return 0;
  }*/
void setLight(byte x) {
  destination_light = x;
  printf("setLight: %d\n", x);
  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.stop();
  delay(10);

  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.llWrite(byte(0x00));
  i2c.llWrite(byte(0x02));
  i2c.repeatedStart(0x51, SoftWire::writeMode);
  i2c.llWrite(x);
  i2c.stop();
  delay(10);

  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.llWrite(byte(0x00));
  i2c.llWrite(byte(0x04));
  i2c.repeatedStart(0x51, SoftWire::writeMode);
  i2c.llWrite(0xaa);
  i2c.stop();
}

uint8_t getLight() {

  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.llWrite(byte(0x00));
  i2c.llWrite(byte(0x05));
  i2c.repeatedStart(0x51, SoftWire::readMode);
  uint8_t c = 0;
  i2c.llRead(c, false); // Dont send ack
  i2c.stop();

  if (c > 19)
    c=0;
  return c;

  /*
     Wire.beginTransmission(0xA2 >> 1);
    Wire.write(byte(0x00));
    Wire.write(byte(0x05));
    Wire.endTransmission();
    Wire.requestFrom(0xA3 >> 1, 1);
    byte c = Wire.read();*/

}


void reconnect() {
  if (timeout.isExpired()) {
  
    // Loop until we're reconnected
    if (!client.connected()) {
      Serial.println("INFO: Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("INFO: connected");
        // Once connected, publish an announcement...
        publishLightState();
        client.subscribe(MQTT_LIGHT_COMMAND_TOPIC);
        client.subscribe(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
      } else {
        Serial.print("ERROR: failed, rc=");
        Serial.print(client.state());
        Serial.println("DEBUG: try again in 5 seconds");
      }
    }

    // Check again in 5 seconds
    timeout.start(5000, AsyncDelay::MILLIS);
  } 
}

int count = 0;
int count2 = 0;
void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  /*
    if (Serial.available()) {
    byte b1 = hex2int(Serial.read());
    byte b2 = hex2int(Serial.read());
    byte f = (b1 << 4) + b2;
    Serial.print("Target value: ");
    Serial.print(f, HEX);
    Serial.println("");
    setLight(f);

    }*/

  static int t = 0;
  for (t = std::max(t, std::max(0, packet_count - 100)); t < packet_count; t++) {
    i2c_soft_slave::print_packet(t, &packets[t % 100]);
  }
  if (set_light != destination_light && set_light != -1) {
    setLight(set_light);
    set_light = -1;
  }

  if (!ota_running && count ++ > 100000) {
    int new_light = getLight();
    if (new_light != light || count2 ++ > 30) {
      light = new_light;
      printf("Light value: %d\n", light);
      if (client.connected())
        publishLightState();
      count2 = 0;
    }
    count = 0;
  }


  reconnect();
  
  client.loop();
  
}
