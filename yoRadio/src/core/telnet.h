#ifndef telnet_h
#define telnet_h

#include <WiFi.h>

#define MAX_TLN_CLIENTS 5
#define MAX_PRINTF_LEN BUFLEN+50

#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i) \
    (((i) & 0x80) ? '1' : '0'), \
    (((i) & 0x40) ? '1' : '0'), \
    (((i) & 0x20) ? '1' : '0'), \
    ((i) & 0x10 ? '1' : '0'), \
    ((i) & 0x08 ? '1' : '0'), \
    ((i) & 0x04 ? '1' : '0'), \
    ((i) & 0x02 ? '1' : '0'), \
    ((i) & 0x01 ? '1' : '0')


class Telnet {
  public:
    Telnet() {};
    bool begin(bool quiet=false);
    void loop();
    void stop();
    void print(uint8_t id, const char *buf);
    void print(const char *buf);
    void printf(uint8_t id, const char *format, ...);
    void printf(const char *format, ...);
    void cleanupClients();
    void info();
  protected:
    WiFiServer server = WiFiServer(23);
    WiFiClient clients[MAX_TLN_CLIENTS];
    void emptyClientStream(WiFiClient client);
    void on_connect(const char* str, uint8_t clientId);
    void on_input(const char* str, uint8_t clientId);
  private:
    bool _isIPSet(IPAddress ip);
    void handleSerial();
};

extern Telnet telnet;

#endif
