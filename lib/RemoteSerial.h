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
