#ifndef config_h
#define config_h
#include "Arduino.h"
#include <Ticker.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <EEPROM.h>
#include <cstddef>      // offsetof, for CONFIG_LAYOUT below
//#include "SD.h"
#include "options.h"
#include "rtcsupport.h"
#include "../pluginsManager/pluginsManager.h"

// EEPROM_START_IR and EEPROM_SIZE are defined further down, after config_t:
// EEPROM_SIZE is derived from sizeof(config_t) so the emulated EEPROM grows and
// shrinks with the struct instead of needing a manual bump.
#define EEPROM_START_IR   0
#ifndef BUFLEN
  #define BUFLEN            170
#endif
#define PLAYLIST_PATH     "/data/playlist.csv"
#define SSIDS_PATH        "/data/wifi.csv"
#define TMP_PATH          "/data/tmpfile.txt"
#define INDEX_PATH        "/data/index.dat"

#define PLAYLIST_SD_PATH     "/data/playlistsd.csv"
#define INDEX_SD_PATH        "/data/indexsd.dat"

#ifdef DEBUG_V
#define DBGH()       { Serial.printf("[%s:%s:%d] Heap: %d\n", __PRETTY_FUNCTION__, __FILE__, __LINE__, xPortGetFreeHeapSize()); }
#define DBGVB( ... ) { char buf[200]; sprintf( buf, __VA_ARGS__ ) ; Serial.print("[DEBUG]\t"); Serial.println(buf); }
#else
#define DBGVB( ... )
#define DBGH()
#endif
#define BOOTLOG( ... ) { char buf[120]; sprintf( buf, __VA_ARGS__ ) ; Serial.print("##[BOOT]#\t"); Serial.println(buf); }
#define EVERY_MS(x)  static uint32_t tmr; bool flag = millis() - tmr >= (x); if (flag) tmr += (x); if (flag)
#define REAL_PLAYL   getMode()==PM_WEB?PLAYLIST_PATH:PLAYLIST_SD_PATH
#define REAL_INDEX   getMode()==PM_WEB?INDEX_PATH:INDEX_SD_PATH

#define MAX_PLAY_MODE   1
#define WEATHERKEY_LENGTH 58
#define MDNS_LENGTH 24
/* mqtt_t sizing. The topic is a prefix that gets "/status", "/command" etc.
   appended, so it must be long enough for a nested topic plus a suffix but
   short enough to stay inside mqtt.cpp's 140-byte scratch buffer. */
#define MQTT_HOST_LENGTH 32
#define MQTT_TOPIC_LENGTH 48
#define MQTT_USER_LENGTH 32
#define MQTT_PASS_LENGTH 32

#if SDC_CS!=255
  #define USE_SD
#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define ESP_ARDUINO_3 1
#endif
#define CONFIG_VERSION  8

enum playMode_e      : uint8_t  { PM_WEB=0, PM_SDCARD=1 };
enum BitrateFormat { BF_UNCNOWN, BF_MP3, BF_AAC, BF_FLAC, BF_OGG, BF_WAV };

void u8fix(char *src);

struct theme_t {
  uint16_t background;
  uint16_t meta;
  uint16_t metabg;
  uint16_t metafill;
  uint16_t title1;
  uint16_t title2;
  uint16_t digit;
  uint16_t div;
  uint16_t weather;
  uint16_t vumax;
  uint16_t vumin;
  uint16_t clock;
  uint16_t clockbg;
  uint16_t seconds;
  uint16_t dow;
  uint16_t date;
  uint16_t heap;
  uint16_t buffer;
  uint16_t ip;
  uint16_t vol;
  uint16_t rssi;
  uint16_t bitrate;
  uint16_t volbarout;
  uint16_t volbarin;
  uint16_t plcurrent;
  uint16_t plcurrentbg;
  uint16_t plcurrentfill;
  uint16_t playlist[5];
};

// MQTT broker settings, editable from the web UI / telnet. Seeded from
// mqttoptions.h on reset, then owned by the user. An empty host disables MQTT
// entirely (no connect attempt, no reconnect timer), so clearing the field is
// how you turn it off without reflashing. An empty user means "no credentials".
struct mqtt_t
{
    char     host[MQTT_HOST_LENGTH];   // broker hostname or IP; "" = disabled
    uint16_t port;                     // 1..65535
    char     topic[MQTT_TOPIC_LENGTH]; // root topic, e.g. "yoradio/lab/"
    char     user[MQTT_USER_LENGTH];   // "" = connect anonymously
    char     pass[MQTT_PASS_LENGTH];

    // so config.saveValue(&store.mqtt, ...) can skip redundant writes
    bool operator==(const mqtt_t &o) const {
        return memcmp(this, &o, sizeof(mqtt_t)) == 0;
    }
};

#ifdef ES8388_ENABLE
// Runtime-adjustable ES8388 settings. These are seeded from myoptions.h on
// factory reset / version upgrade, and afterwards the web UI and telnet
// overwrite them and persist them, so a reboot keeps the user's choice.
struct es8388_t
{
    // --- volumes ---
    uint8_t es_master_vol;   // UNUSED: kept only so sizeof(config_t) and the EEPROM
                             // layout stay stable. The main-page volume drives the
                             // DAC master register directly (Player::applyEs8388Volume).
    uint8_t es_vol1;         // 0..33 LOUT1/ROUT1 analog volume, 30 = 0dB
    uint8_t es_vol2;         // 0..33 LOUT2/ROUT2 analog volume, 30 = 0dB
    int8_t  es_bal1;         // -6..+6 LOUT1/ROUT1 L/R balance
    int8_t  es_bal2;         // -6..+6 LOUT2/ROUT2 L/R balance
    // --- DAC Control 7 (0x1d) ---
    uint8_t es_stereo_eff;   // 0..7 stereo enhancement
    uint8_t es_mono;         // 0 = stereo, 1 = (L+R)/2
    uint8_t es_vpp;          // 0..3 DAC Vpp scale (3.5/4.0/3.0/2.5 V)
    // --- DAC Control 3 (0x19) ---
    uint8_t es_soft_ramp;    // 1 = soft volume ramp (removes clicks)
    uint8_t es_ramp_rate;    // 0..3 ramp rate selector
    // --- DAC Control 6 (0x1c) ---
    uint8_t es_deemph;       // 0 off, 1 = 32k, 2 = 44.1k, 3 = 48k
    uint8_t es_clickfree;    // 1 = click-free power up/down
    uint8_t es_invl;         // invert left channel phase
    uint8_t es_invr;         // invert right channel phase
    // --- DAC Control 23 (0x2d) ---
    uint8_t es_vroi;         // 0 = 1.5k output impedance, 1 = 40k
    // --- output mixers (0x27/0x2a) ---
    uint8_t es_linein;       // 1 = mix LIN1/LIN2 into the outputs
    int8_t  es_linein_gain;  // -15..+6 dB, 3dB steps
    // --- ADC (0x03/0x09/0x0a) ---
    uint8_t es_adc;          // 1 = power up the ADC
    uint8_t es_mic_pga;      // 0..8 = 0..+24dB in 3dB steps
    uint8_t es_mic_sel;      // 0 = LIN1&RIN1, 1 = LIN2&RIN2, 2 = differential
    uint8_t es_mic_bias;     // 1 = MBIAS output on (electret mics)
    // --- power management ---
    uint8_t es_standby;      // 1 = codec standby when playback stops
    // --- output mute ---
    uint8_t es_mute1;        // 1 = mute LOUT1/ROUT1
    uint8_t es_mute2;        // 1 = mute LOUT2/ROUT2
    uint8_t es_mute_main;    // 1 = digital mute

    // so config.saveValue(&store.es8388, ...) can skip redundant writes
    bool operator==(const es8388_t &o) const {
        return memcmp(this, &o, sizeof(es8388_t)) == 0;
    }
};

struct config_t
{
  uint16_t  config_set; //must be 4262
  uint16_t  version;
  // Fingerprint of the struct layout this firmware expects (see
  // CONFIG_LAYOUT below). Stored so a reflash that changes the layout is
  // detected and the settings are re-seeded, instead of silently reading
  // the old bytes as if they were still meaningful.
  uint16_t  layout;
  uint8_t   volume;
  int8_t    balance;
  int8_t    trebble;
  int8_t    middle;
  int8_t    bass;
  uint16_t  lastStation;
  uint16_t  countStation;
  uint8_t   lastSSID;
  bool      audioinfo;
  uint8_t   smartstart;
  int8_t    tzHour;
  int8_t    tzMin;
  uint16_t  timezoneOffset;
  bool      vumeter;
  uint8_t   softapdelay;
  bool      flipscreen;
  bool      invertdisplay;
  bool      numplaylist;
  bool      fliptouch;
  bool      dbgtouch;
  bool      dspon;
  uint8_t   brightness;
  uint8_t   contrast;
  char      sntp1[35];
  char      sntp2[35];
  bool      showweather;
  char      weatherlat[10];
  char      weatherlon[10];
  char      weatherkey[WEATHERKEY_LENGTH];
  uint16_t  _reserved;
  uint16_t  lastSdStation;
  bool      sdsnuffle;
  uint8_t   volsteps;
  uint16_t  encacc;
  uint8_t   play_mode;  //0 WEB, 1 SD
  uint8_t   irtlp;
  bool      btnpullup;
  uint16_t  btnlongpress;
  uint16_t  btnclickticks;
  uint16_t  btnpressticks;
  bool      encpullup;
  bool      enchalf;
  bool      enc2pullup;
  bool      enc2half;
  bool      forcemono;
  bool      i2sinternal;
  bool      rotate90;
  bool      screensaverEnabled;
  uint16_t  screensaverTimeout;
  bool      screensaverBlank;
  bool      screensaverPlayingEnabled;
  uint16_t  screensaverPlayingTimeout;
  bool      screensaverPlayingBlank;
  char      mdnsname[24];
  bool      skipPlaylistUpDown;
  // user-controlled speaker/amp mute. uint8_t, not bool: a bool read straight
  // from EEPROM cannot be told apart from a 0xFF junk byte, and junk here would
  // silently leave the amp muted. Non-zero means muted.
  uint8_t   spmute;
#ifdef ES8388_ENABLE
  es8388_t  es8388;  // runtime ES8388 settings, seeded from myoptions.h
#endif
  mqtt_t    mqtt;    // broker host/port/root topic, seeded from mqttoptions.h
};

/* Where config_t lives in the emulated EEPROM.
   ircodes_t sits at EEPROM_START_IR and is 484 bytes, so when IR is enabled the
   config has to start past it. When IR is disabled that block is not compiled
   at all, so there is no reason to reserve room for it. */
#if IR_PIN!=255
  #define EEPROM_START 500
#else
  #define EEPROM_START 0
#endif

/* The ESP32 has no real EEPROM: this is a RAM buffer that the Arduino core
   flushes to NVS as a single blob. Sizing it from the struct means the region
   tracks config_t automatically, so adding a setting can no longer overrun a
   hardcoded limit. The slack leaves room to grow the struct without changing
   this expression. */
#define EEPROM_SIZE       (EEPROM_START + (int)sizeof(config_t) + 32)

/* Layout fingerprint, stored in config_t::layout.
   Built from the offsets of a few stable fields rather than sizeof(), so that
   APPENDING to the struct (the normal way to add a setting) does not trip it,
   while a field reorder or a #ifdef-gated group appearing/disappearing in the
   middle of the struct does. Those would otherwise silently reinterpret old
   EEPROM bytes. EEPROM_START is folded in because changing it moves everything.
   The canaries are unconditional fields that are not likely to be removed. */
#define CONFIG_LAYOUT     ((uint16_t)( \
     3u * offsetof(config_t, volume) \
   + 5u * offsetof(config_t, sntp1) \
   + 7u * offsetof(config_t, btnlongpress) \
   + 11u * offsetof(config_t, mdnsname) \
   + 13u * (unsigned)EEPROM_START ))

#if IR_PIN!=255
struct ircodes_t
{
  unsigned int ir_set; //must be 4224
  uint64_t irVals[20][3];
};
#endif

#endif

struct station_t
{
  char name[BUFLEN];
  char url[BUFLEN];
  char title[BUFLEN];
  uint16_t bitrate;
  int  ovol;
};

struct neworkItem
{
  char ssid[30];
  char password[40];
};

class Config {
  public:
    config_t store;
    station_t station;
    theme_t   theme;
#if IR_PIN!=255
    int irindex;
    uint8_t irchck;
    ircodes_t ircodes;
#endif
    BitrateFormat configFmt = BF_UNCNOWN;
    neworkItem ssids[5];
    uint8_t ssidsCount;
    uint16_t sleepfor;
    uint32_t sdResumePos;
    bool     emptyFS;
    uint16_t vuThreshold;
    uint16_t screensaverTicks;
    uint16_t screensaverPlayingTicks;
    bool     isScreensaver;
  public:
    Config() {};
    //void save();
#if IR_PIN!=255
    void saveIR();
#endif
    void init();
    void loadTheme();
    uint8_t setVolume(uint8_t val);
    void saveVolume();
    void setTone(int8_t bass, int8_t middle, int8_t trebble);
    void setBalance(int8_t balance);
    uint8_t setLastStation(uint16_t val);
    uint8_t setCountStation(uint16_t val);
    uint8_t setLastSSID(uint8_t val);
    void setTitle(const char* title);
    void setStation(const char* station);
    bool parseCSV(const char* line, char* name, char* url, int &ovol);
    bool parseJSON(const char* line, char* name, char* url, int &ovol);
    bool parseWsCommand(const char* line, char* cmd, char* val, uint8_t cSize);
    bool parseSsid(const char* line, char* ssid, char* pass);
    void loadStation(uint16_t station);
    bool initNetwork();
    bool saveWifi();
    bool saveWifiFromNextion(const char* post);
    void setSmartStart(uint8_t ss);
    void setBitrateFormat(BitrateFormat fmt) { configFmt = fmt; }
    void initPlaylist();
    void indexPlaylist();
    #ifdef USE_SD
      void initSDPlaylist();
      void changeMode(int newmode=-1);
    #endif
    uint16_t lastStation(){
      return getMode()==PM_WEB?store.lastStation:store.lastSdStation;
    }
    void lastStation(uint16_t newstation){
      if(getMode()==PM_WEB) saveValue(&store.lastStation, newstation);
      else saveValue(&store.lastSdStation, newstation);
    }
    uint8_t fillPlMenu(int from, uint8_t count, bool fromNextion=false);
    char * stationByNum(uint16_t num);
    void setTimezone(int8_t tzh, int8_t tzm);
    void setTimezoneOffset(uint16_t tzo);
    uint16_t getTimezoneOffset();
    void setBrightness(bool dosave=false);
    void setDspOn(bool dspon, bool saveval = true);
    void setSpeakerMute(bool muted);
#ifdef ES8388_ENABLE
    /* Seed store.es8388 from the myoptions.h compile-time defaults. */
    void setEs8388Defaults();
#endif
    /* Seed store.mqtt from the mqttoptions.h compile-time defaults. */
    void setMqttDefaults();
    /* True when a broker host is configured, i.e. MQTT should run. */
    bool mqttEnabled() const { return store.mqtt.host[0] != '\0'; }
    void sleepForAfter(uint16_t sleepfor, uint16_t sleepafter=0);
    void bootInfo();
    void doSleepW();
    void setSnuffle(bool sn);
    uint8_t getMode() { return store.play_mode/* & 0b11*/; }
    void initPlaylistMode();
    void reset();
    bool spiffsCleanup();
    FS* SDPLFS(){ return _SDplaylistFS; }
    #if RTCSUPPORTED
      bool isRTCFound(){ return _rtcFound; };
    #endif
    template <typename T>
    size_t getAddr(const T *field) const {
      return (size_t)((const uint8_t *)field - (const uint8_t *)&store) + EEPROM_START;
    }
    template <typename T>
    void saveValue(T *field, const T &value, bool commit=true, bool force=false){
      if(*field == value && !force) return;
      *field = value;
      size_t address = getAddr(field);
      EEPROM.put(address, value);
      if(commit)
        EEPROM.commit();
    }
    void saveValue(char *field, const char *value, size_t N, bool commit=true, bool force=false) {
      if (strcmp(field, value) == 0 && !force) return;
      strlcpy(field, value, N);
      size_t address = getAddr(field);
      size_t fieldlen = strlen(field);
      for (size_t i = 0; i <=fieldlen ; i++) EEPROM.write(address + i, field[i]);
      if(commit)
        EEPROM.commit();
    }
    uint32_t getChipId(){
      uint32_t chipId = 0;
      for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
      }
      return chipId;
    }
  private:
    template <class T> int eepromWrite(int ee, const T& value);
    template <class T> int eepromRead(int ee, T& value);
    bool _bootDone;
    #if RTCSUPPORTED
      bool _rtcFound;
    #endif
    FS* _SDplaylistFS;
    void setDefaults();
    Ticker   _sleepTimer;
    static void doSleep();
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
    void _setupVersion();
    void _initHW();
    bool _isFSempty();
    uint16_t _randomStation(){
      randomSeed(esp_random() ^ millis());
      uint16_t station = random(1, store.countStation);
      return station;
    }
    char _stationBuf[BUFLEN/2];
};

extern Config config;
#if DSP_HSPI || TS_HSPI || VS_HSPI
extern SPIClass  SPI2;
#endif

#endif
