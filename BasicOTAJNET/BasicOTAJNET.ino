#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPUpdateServer.h>
#include <SoftWire.h>

const char* ssid = "JNET";
const char* password = "";
const char *software_version = "3.1";

const uint8_t sdaPin = 2;
const uint8_t sclPin = 0;
SoftWire i2c(sdaPin, sclPin);
int ota_running = 0;
uint8_t light = 0;
void setLight(byte x);


class RemoteSerialUDP {
public:
  WiFiUDP Udp;
  IPAddress ip;

  RemoteSerialUDP() {
    ip.fromString("192.168.1.167");
  };

  void printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start (args, format);
    vsnprintf (buf,sizeof(buf),format, args);
    va_end(args);
    Udp.beginPacket(ip, 1234);
    Udp.write(buf);
    Udp.endPacket();
  }
};
class RemoteSerialTCP {
public:
  WiFiClient Tcp;
  IPAddress ip;

  RemoteSerialTCP() {
    ip.fromString("192.168.1.167");
  };

  void printf(const char *format, ...) {
    if (!Tcp.connected()) {
      Tcp.connect(ip, 1234);
    }
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf,sizeof(buf), format, args);
    va_end(args);
    Tcp.print(buf);
  }
};

ESP8266WebServer server ( 80 );
ESP8266HTTPUpdateServer httpUpdater;
RemoteSerialTCP RSerial;

class i2c_soft_slave {
public:
  
  const int slave_sda = 3;
  const int slave_scl = 1;
  
  enum i2c_state {
    STOP = 0,
    START = 1,
    ADDR_0 = 2,
    ADDR_1 = 3,
    ADDR_2 = 4,
    ADDR_3 = 5,
    ADDR_4 = 6,
    ADDR_5 = 7,
    ADDR_6 = 8,
    RW_BIT = 9,
    ADDR_ACK = 10,
    DATA_0 = 11,
    DATA_1 = 12,
    DATA_2 = 13,
    DATA_3 = 14,
    DATA_4 = 15,
    DATA_5 = 16,
    DATA_6 = 17,
    DATA_7 = 18,
    DATA_ACK = 19
  };

  volatile i2c_state state = STOP;

  struct i2c_packet {
    int count;
    int addr;
    int w;
    int dc;
    int data[5];
  };
  
  volatile int p = -1;
  volatile i2c_packet packets[100];
  volatile i2c_packet *pack;

  void setInput(int input) {
    if (input) {
      pinMode(slave_sda, INPUT);
    } else {
      pinMode(slave_sda, OUTPUT);
    }
  }

  void stop() {
    setInput(true);
    this->state = STOP;
    if (pack && pack->w) {
      on_write((i2c_packet *)pack);
    }
    pack = nullptr;
   
  }
  
  void data_fall() {

    // If data falls during CLCK high, its a start condition.
    if (/*this->state == STOP && */digitalRead(slave_scl) == 1) {

      // RESET counter
      this->state = START;

      // Count up package
      p++;

      // Setup next package      
      pack = &packets[p % 100];        
      memset((void *)pack, 0, sizeof(i2c_packet));
      pack->count = p;
    }
  }
  
  void clock_rise() {
    if (this->state == STOP)
      return;

    if (!pack)
      return;
    
    // If no activa packet, eller to many bytes, just make sure we are in stopped state.
    if (pack->dc > 5) {
      stop();
      return;
    }
          
    switch (this->state) {
      // First 7 bits is the address
      case ADDR_0:
      case ADDR_1:
      case ADDR_2:
      case ADDR_3:
      case ADDR_4:
      case ADDR_5:
      case ADDR_6:
        pack->addr <<= 1;
        pack->addr |= digitalRead(slave_sda);
        break;

      // Read rw-bit
      case RW_BIT:
        pack->w = !digitalRead(slave_sda);
        break;

      // If in read mode, read 8 bytes
      case DATA_0:
      case DATA_1:
      case DATA_2:
      case DATA_3:
      case DATA_4:
      case DATA_5:
      case DATA_6:
      case DATA_7:
        if (pack->w) {
          pack->data[pack->dc] <<= 1;
          pack->data[pack->dc] |= digitalRead(slave_sda);
        }
        break;
    }
    
  }
  
  void clock_fall() {
    if (this->state == STOP)
      return;

    if (!pack)
      return;
    
    switch (this->state) {
      // On r/w bit fall, pull data low as next one is an ACK
      case RW_BIT:
        setInput(false);    
        digitalWrite(slave_sda, 0);
        break;

      // On falling clock after sending ACK bit, just release data bus.
      case ADDR_ACK:
         // If master is wiring
         if (pack->w) {
          setInput(true);

         // If master is reading
         } else {
          setInput(false);
          this->on_read((i2c_packet *)pack); // Lett callback fill the data...
          digitalWrite(slave_sda, (pack->data[pack->dc] & 0x80) != 0); // Write first byte
        }
        break;
        
      // If in write mode, write 7 more bytes
      case DATA_0:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x40) != 0); break;
      case DATA_1:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x20) != 0); break;
      case DATA_2:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x10) != 0); break;
      case DATA_3:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x8) != 0); break;
      case DATA_4:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x4) != 0); break;
      case DATA_5:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x2) != 0); break;
      case DATA_6:
        if (!pack->w) digitalWrite(slave_sda, (pack->data[pack->dc] & 0x1) != 0); break;

      // When DATA_7 Falls, we are done, and prepares for ack
      case DATA_7:
        if (pack->w) {
          setInput(false);
          digitalWrite(slave_sda, 0);
        } else {
          setInput(true);
        }
        pack->dc++;
        break;

                
      // On ack fall, prepare for send or read
      case DATA_ACK:
        // Reading from device.
        if (!pack->w) {
          if (digitalRead(slave_sda)) {
            stop();
            break;
          }
          setInput(false);
          digitalWrite(slave_sda, pack->data[pack->dc] & 0x80); // Write the first bit of next byte
          pack->data[pack->dc] = pack->data[pack->dc] << 1;          
        // Writing to device.
        } else {
          setInput(true);
          if (digitalRead(slave_sda)) {
            stop();
          }
        }
        break;
    }

    // If last data bit, the next bit will be handled the same as an ACK.
    if (this->state == DATA_ACK) {
      this->state = DATA_0;
    } else {
      int x = (int)this->state;
      x++;
      this->state = (i2c_state)x;
    }

  }
  
  int t=0;
  void pr() {
    for (; t < p; t++) {
      i2c_packet *pack = (i2c_packet *)&packets[t % 100];  
      if (pack->count != t)
        continue;
      RSerial.printf("count: %04d %s addr: 0x%02x data: ", pack->count, pack->w ? "WRITE" : "READ ", 
        pack->addr);
      for (int i = 0; i < pack->dc; i++)
         RSerial.printf("0x%02x ", pack->data[i]);
      RSerial.printf("\n");
      
    }
  }

  void (*on_read)(i2c_packet *pack);
  void (*on_write)(i2c_packet *pack);

  static i2c_soft_slave *lthis;
  i2c_soft_slave(void (*on_read_)(i2c_packet *pack), void (*on_write_)(i2c_packet *pack)) : on_read(on_read_), on_write(on_write_) {
    pinMode(slave_sda, INPUT);
    pinMode(slave_scl, INPUT);
    pinMode(sdaPin, OUTPUT);
    pinMode(sclPin, OUTPUT);

    setInput(true);
    lthis = this;
    attachInterrupt(digitalPinToInterrupt(slave_sda), [](){
      if (!digitalRead(lthis->slave_sda))
        i2c_soft_slave::lthis->data_fall();
    }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(slave_scl), [](){
      if (digitalRead(lthis->slave_scl))
        i2c_soft_slave::lthis->clock_rise();
      else
        i2c_soft_slave::lthis->clock_fall();
    }, CHANGE);
    
  }
};
i2c_soft_slave *i2c_soft_slave::lthis = nullptr;

volatile int set_light = -1;
void i2c_read(i2c_soft_slave::i2c_packet *pack) {
  if (pack->addr == 0x51) {
    pack->data[0] = light;
  }
}

void i2c_write(i2c_soft_slave::i2c_packet *pack) {
  if (pack->addr == 0x51) {
     set_light = pack->data[1];
  }
 
}

i2c_soft_slave i2c_slave(&i2c_read, &i2c_write);

void setup() {
  //Serial.begin(115200);
  //Serial.println("Booting New 3.0");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    //Serial.print ( "." );
  }

  RSerial.printf( "Connected to: %s\n", ssid);
  RSerial.printf( "IP address: %s\n", WiFi.localIP().toString().c_str());

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("esp8266-wakeup");
  ArduinoOTA.onStart([]() {
    RSerial.printf("Start\n");
    ota_running = 1;
  });
  ArduinoOTA.onEnd([]() {
    RSerial.printf("\nEnd\n");
    ota_running = 0;
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    RSerial.printf("Progress: %u%%\r", (progress / (total / 100)));
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
  RSerial.printf("Ready\n");

  if ( MDNS.begin( "esp8266-wakeup" ) ) {
    RSerial.printf( "MDNS responder started\n" );
  }


  i2c.setDelay_us(40);
  i2c.setTimeout_ms(40);
  i2c.begin();

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

  RSerial.printf("setLight: %d\n", x);
  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.stop();
  delay(10);
  
  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.write(byte(0x00)); 
  i2c.write(byte(0x02));
  i2c.repeatedStart(0x51, SoftWire::writeMode);
  i2c.write(x);
  i2c.stop();
  delay(10);

  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.write(byte(0x00));
  i2c.write(byte(0x04));
  i2c.repeatedStart(0x51, SoftWire::writeMode);
  i2c.write(0xaa);
  i2c.stop();
  delay(10);
}

uint8_t getLight() {

  i2c.startWait(0x51, SoftWire::writeMode);
  i2c.write(byte(0x00));
  i2c.write(byte(0x05));
  i2c.repeatedStart(0x51, SoftWire::readMode);
  uint8_t c = 0;
  i2c.read(c, false);
  i2c.stop();
  return c;
    
/*
   Wire.beginTransmission(0xA2 >> 1);
  Wire.write(byte(0x00));
  Wire.write(byte(0x05));
  Wire.endTransmission();
  Wire.requestFrom(0xA3 >> 1, 1);
  byte c = Wire.read();*/

}

int count = 0;
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

  i2c_slave.pr();
  if (set_light != -1) {
    setLight(set_light);
    set_light = -1;
  }
  
  if (!ota_running && count ++ > 100000) {
    light = getLight();
    RSerial.printf("Light value: %d state: %d p: %d\n", light, i2c_slave.state, i2c_slave.p);
    
     //RSerial.printf("read : sda: %d scl: %d\n", digitalRead(slave_sda), digitalRead(slave_scl));
    count = 0;
  }
}
