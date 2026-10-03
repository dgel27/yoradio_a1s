#include <stdarg.h>
#include "WiFi.h"

#include "config.h"
#include "player.h"
#include "network.h"
#include "telnet.h"
#if ES8388_ENABLE
  #include "../audioES8388/ES8388.h"
#endif // ES8388_ENABLE
#ifdef MQTT_ROOT_TOPIC
  #include "mqtt.h"
#endif // MQTT_ROOT_TOPIC

Telnet telnet;

bool Telnet::_isIPSet(IPAddress ip) {
  return ip.toString() == "0.0.0.0";
}

bool Telnet::begin(bool quiet) {
  if(network.status==SDREADY) {
    BOOTLOG("Ready in SD Mode!");
    BOOTLOG("------------------------------------------------");
    Serial.println("##[BOOT]#");
    return true;
  }
  if(!quiet) Serial.print("##[BOOT]#\ttelnet.begin\t");
  if (WiFi.status() == WL_CONNECTED || _isIPSet(WiFi.softAPIP())) {
    server.begin();
    server.setNoDelay(true);
    if(!quiet){
      Serial.println("done");
      Serial.println("##[BOOT]#");
      BOOTLOG("Ready! Go to http:/%s/ to configure", WiFi.localIP().toString().c_str());
      BOOTLOG("------------------------------------------------");
      Serial.println("##[BOOT]#");
    }
    return true;
  } else {
    return false;
  }
}

void Telnet::stop() {
  server.stop();
}

void Telnet::emptyClientStream(WiFiClient client) {
  client.flush();
  delay(50);
  while (client.available()) {
    client.read();
  }
}

void Telnet::cleanupClients() {
  for (int i = 0; i < MAX_TLN_CLIENTS; i++) {
    if (!clients[i].connected()) {
      if (clients[i]) {
        Serial.printf("Client [%d] is %s\n", i, clients[i].connected() ? "connected" : "disconnected");
        clients[i].stop();
      }
    }
  }
}

void Telnet::handleSerial(){
  if(Serial.available()){
    String request = Serial.readStringUntil('\n'); request.trim();
    on_input(request.c_str(), 100);
  }
}

void Telnet::loop() {
  if(network.status==SDREADY || network.status!=CONNECTED) {
    handleSerial();
    return;
  }
  uint8_t i;
  if (WiFi.status() == WL_CONNECTED) {
    if (server.hasClient()) {
      for (i = 0; i < MAX_TLN_CLIENTS; i++) {
        if (!clients[i] || !clients[i].connected()) {
          if (clients[i]) {
            clients[i].stop();
          }
          clients[i] = server.available();
          if (!clients[i]) Serial.println("available broken");
          on_connect(clients[i].remoteIP().toString().c_str(), i);
          clients[i].setNoDelay(true);
          emptyClientStream(clients[i]);
          break;
        }
      }
      if (i >= MAX_TLN_CLIENTS) {
        server.available().stop();
      }
    }
    for (i = 0; i < MAX_TLN_CLIENTS; i++) {
      if (clients[i] && clients[i].connected() && clients[i].available()) {
        String inputstr = clients[i].readStringUntil('\n');
        inputstr.trim();
        on_input(inputstr.c_str(), i);
      }
    }
  } else {
    for (i = 0; i < MAX_TLN_CLIENTS; i++) {
      if (clients[i]) {
        clients[i].stop();
      }
    }
    delay(1000);
  }
  handleSerial();
  yield();
}

void Telnet::print(const char *buf) {
  for (int id = 0; id < MAX_TLN_CLIENTS; id++) {
    if (clients[id] && clients[id].connected()) {
      print(id, buf);
    }
  }
  Serial.print(buf);
}

void Telnet::print(uint8_t id, const char *buf) {
  if (clients[id] && clients[id].connected()) {
    clients[id].print(buf);
  }
}

void Telnet::printf(const char *format, ...) {
  char buf[MAX_PRINTF_LEN];
  va_list args;
  va_start (args, format );
  vsnprintf(buf, MAX_PRINTF_LEN, format, args);
  va_end (args);
  for (int id = 0; id < MAX_TLN_CLIENTS; id++) {
    if (clients[id] && clients[id].connected()) {
      clients[id].print(buf);
    }
  }
  if (strcmp(buf, "> ") == 0) return;
  //if(strstr(buf,"\n> ")==NULL) Serial.print(buf);
  char *nl = strstr(buf, "\n> ");
  if (nl != NULL) { buf[nl-buf+1] = '\0'; }
  Serial.print(buf);
}

void Telnet::printf(uint8_t id, const char *format, ...) {
  char buf[MAX_PRINTF_LEN];
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buf, MAX_PRINTF_LEN, format, argptr);
  va_end(argptr);
  if(id>MAX_TLN_CLIENTS){
    Serial.print(buf);
    return;
  }
  if (clients[id] && clients[id].connected()) {
    clients[id].print(buf);
  }
}

void Telnet::disconnectClient(uint8_t clientId) {
  if (clientId >= MAX_TLN_CLIENTS) return;
  if (clients[clientId]) {
    clients[clientId].stop();
  }
}

void Telnet::on_connect(const char* str, uint8_t clientId) {
  Serial.printf("Telnet: [%d] %s connected\n", clientId, str);
  print(clientId, "\nWelcome to ёRadio!\n(Use ^] + q  to disconnect. Type 'help' for commands.)\n> ");
}

void Telnet::printHelp(uint8_t clientId) {
  printf(clientId, "Available commands:\n");
  printf(clientId, "  help                   Show this help\n");
  printf(clientId, "  quit | bye | exit      Disconnect this session\n");
  printf(clientId, "  prev | next | toggle   Station prev/next, play/pause\n");
  printf(clientId, "  stop | start           Stop / resume playback\n");
  printf(clientId, "  play <n>               Play station number n\n");
  printf(clientId, "  vol                    Show volume\n");
  printf(clientId, "  vol <0-254> | vol+ | vol-  Set / step volume\n");
  printf(clientId, "  info | list            Player info / station list\n");
  printf(clientId, "  audioinfo [0|1]        Show / set audioinfo output\n");
  printf(clientId, "  smartstart [0|1]       Show / set smartstart\n");
  printf(clientId, "  date | time            Sync and show time\n");
  printf(clientId, "  tzo [h[:m]]            Show / set timezone offset\n");
  printf(clientId, "  dspon <0|1>            Display on/off\n");
  printf(clientId, "  dim <0-100>            Display brightness\n");
  printf(clientId, "  sleep <for> [after]    Sleep timer, minutes\n");
  #ifdef USE_SD
  printf(clientId, "  mode <0|1|2>           0=WEB, 1=SD card, 2=toggle\n");
  #endif
  printf(clientId, "  version | heap         Firmware version / free heap\n");
  printf(clientId, "  wifi                   Scan networks\n");
  printf(clientId, "  wifi.status | wifi.rssi  Connection status / signal\n");
  printf(clientId, "  wifi.con | wifi.station  Saved networks / current\n");
  printf(clientId, "  wifi <ssid> <pass>      Save network, reboot\n");
  printf(clientId, "  discon                 Disconnect wifi\n");
  printf(clientId, "  boot | reset           Reboot / factory reset (!)\n");
  #if ES8388_ENABLE
  printf(clientId, "  esvol <n>               set volume 0-254 (alias of the main-page volume)\n");
  printf(clientId, "  esvol1 <n>              ES8388 OUT1 (speaker) volume 0-33\n");
  printf(clientId, "  esvol2 <n>              ES8388 OUT2 (headphone) volume 0-33\n");
  printf(clientId, "  esch1bal <n>            ES8388 OUT1 L/R balance -6..+6\n");
  printf(clientId, "  esch2bal <n>            ES8388 OUT2 L/R balance -6..+6\n");
  printf(clientId, "  esstereo <n>            ES8388 stereo widening 0-7 (not a tone EQ)\n");
  printf(clientId, "  esvpp <n>               ES8388 DAC Vpp scale 0-3\n");
  printf(clientId, "  esdeemph <n>            ES8388 de-emphasis 0-3\n");
  printf(clientId, "  esramprate <n>          ES8388 soft-ramp rate 0-3\n");
  printf(clientId, "  esramp on|off           ES8388 soft volume ramp\n");
  printf(clientId, "  esvroi on|off           ES8388 output impedance 1.5k/40k\n");
  printf(clientId, "  esclick on|off          ES8388 click-free power up/down\n");
  printf(clientId, "  esinvl on|off           ES8388 invert left channel\n");
  printf(clientId, "  esinvr on|off           ES8388 invert right channel\n");
  printf(clientId, "  eslinein off|mix|line   ES8388 line-in: off, mixed with radio, or line only\n");
  printf(clientId, "  eslingain <dB>          ES8388 line-in gain -15..+6, in 3dB steps\n");
  printf(clientId, "  esadc on|off            ES8388 power up the ADC\n");
  printf(clientId, "  esmicpga <n>            ES8388 mic preamp gain 0-8 (0..+24dB)\n");
  printf(clientId, "  esmicin <n>             ES8388 mic input 0=LIN1 1=LIN2 2=diff\n");
  printf(clientId, "  esmicbias on|off        ES8388 mic bias (MBIAS)\n");
  printf(clientId, "  esstandby on|off        ES8388 standby while stopped\n");
  printf(clientId, "  esreset                 ES8388 settings back to defaults\n");
  printf(clientId, "  esmono on|off           ES8388 mono/stereo\n");
  printf(clientId, "  esspk mute|unmute       ES8388 speaker amp mute\n");
  printf(clientId, "  esmute2 <0|1>           ES8388 headphone amp mute (overridden while unplugged)\n");
  #if HP_DETECT!=255
  printf(clientId, "  hp                     headphone jack state, raw pin level and forced mute\n");
  #endif
  printf(clientId, "  esregr <reg> | esregw <reg> <val> | esdump   register debug\n");
  #endif
  #ifdef MQTT_ROOT_TOPIC
  printf(clientId, "  mqtthost <host|->       MQTT broker host; \"-\" clears it (disables MQTT)\n");
  printf(clientId, "  mqttport <n>            MQTT broker port (default 1883)\n");
  printf(clientId, "  mqtttopic <prefix>      MQTT root topic, e.g. yoradio/lab/\n");
  printf(clientId, "  mqttuser <user|->       MQTT username; \"-\" connects anonymously\n");
  printf(clientId, "  mqttpass <pass|->       MQTT password; \"-\" clears it\n");
  printf(clientId, "  mqttreset               MQTT settings back to mqttoptions.h defaults\n");
  #endif
  printf(clientId, "Most commands also accept the cli. prefix and (args) form.\n> ");
}

void Telnet::info() {
  telnet.printf("##CLI.INFO#\n");
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S+03:00", &network.timeinfo);
  telnet.printf("##SYS.DATE#: %s\n", timeStringBuff); //TODO timezone offset
  telnet.printf("##CLI.NAMESET#: %d %s\n", config.lastStation(), config.station.name);
  if (player.status() == PLAYING) {
    telnet.printf("##CLI.META#: %s\n",  config.station.title);
  }
  telnet.printf("##CLI.VOL#: %d\n", config.store.volume);
  if (player.status() == PLAYING) {
    telnet.printf("##CLI.PLAYING#\n");
  } else {
    telnet.printf("##CLI.STOPPED#\n");
  }
  telnet.printf("> ");
}

void Telnet::on_input(const char* str, uint8_t clientId) {
  if (strlen(str) == 0) return;
  if (strcmp(str, "quit") == 0 || strcmp(str, "bye") == 0 || strcmp(str, "exit") == 0) {
    disconnectClient(clientId);
    return;
  }
  if (strcmp(str, "help") == 0) {
    printHelp(clientId);
    return;
  }
  if(network.status == CONNECTED){
    if (strcmp(str, "cli.prev") == 0 || strcmp(str, "prev") == 0) {
      player.prev();
      return;
    }
    if (strcmp(str, "cli.next") == 0 || strcmp(str, "next") == 0) {
      player.next();
      return;
    }
    if (strcmp(str, "cli.toggle") == 0 || strcmp(str, "toggle") == 0) {
      player.toggle();
      return;
    }
    if (strcmp(str, "cli.stop") == 0 || strcmp(str, "stop") == 0) {
      player.sendCommand({PR_STOP, 0});
      //info();
      return;
    }
    if (strcmp(str, "cli.start") == 0 || strcmp(str, "start") == 0 || strcmp(str, "cli.play") == 0 || strcmp(str, "play") == 0) {
      player.sendCommand({PR_PLAY, config.lastStation()});
      return;
    }
    if (strcmp(str, "cli.vol") == 0 || strcmp(str, "vol") == 0) {
      printf(clientId, "##CLI.VOL#: %d\n> ", config.store.volume);
      return;
    }
    if (strcmp(str, "cli.vol-") == 0 || strcmp(str, "vol-") == 0) {
      player.stepVol(false);
      return;
    }
    if (strcmp(str, "cli.vol+") == 0 || strcmp(str, "vol+") == 0) {
      player.stepVol(true);
      return;
    }
    if (strcmp(str, "sys.date") == 0 || strcmp(str, "date") == 0 || strcmp(str, "time") == 0) {
      network.requestTimeSync(true, clientId > MAX_TLN_CLIENTS?clientId:0);
      return;
    }
    int volume;
    if (sscanf(str, "vol(%d)", &volume) == 1 || sscanf(str, "cli.vol(\"%d\")", &volume) == 1 || sscanf(str, "vol %d", &volume) == 1) {
      if (volume < 0) volume = 0;
      if (volume > 254) volume = 254;
      player.setVol(volume);
      return;
    }
    if (strcmp(str, "cli.audioinfo") == 0 || strcmp(str, "audioinfo") == 0) {
      printf(clientId, "##CLI.AUDIOINFO#: %d\n> ", config.store.audioinfo > 0);
      return;
    }
    int ainfo;
    if (sscanf(str, "audioinfo(%d)", &ainfo) == 1 || sscanf(str, "cli.audioinfo(\"%d\")", &ainfo) == 1 || sscanf(str, "audioinfo %d", &ainfo) == 1) {
      config.saveValue(&config.store.audioinfo, ainfo > 0);
      printf(clientId, "new audioinfo value is: %d\n> ", config.store.audioinfo);
      return;
    }
    if (strcmp(str, "cli.smartstart") == 0 || strcmp(str, "smartstart") == 0) {
      printf(clientId, "##CLI.SMARTSTART#: %d\n> ", config.store.smartstart);
      return;
    }
    int sstart;
    if (sscanf(str, "smartstart(%d)", &sstart) == 1 || sscanf(str, "cli.smartstart(\"%d\")", &sstart) == 1 || sscanf(str, "smartstart %d", &sstart) == 1) {
      config.saveValue(&config.store.smartstart, static_cast<uint8_t>(sstart));
      printf(clientId, "new smartstart value is: %d\n> ", config.store.smartstart);
      return;
    }
    if (strcmp(str, "cli.list") == 0 || strcmp(str, "list") == 0) {
      printf(clientId, "#CLI.LIST#\n");
      File file = SPIFFS.open(PLAYLIST_PATH, "r");
      if (!file || file.isDirectory()) {
        return;
      }
      char sName[BUFLEN], sUrl[BUFLEN];
      int sOvol;
      uint16_t c = 1;
      while (file.available()) {
        if (config.parseCSV(file.readStringUntil('\n').c_str(), sName, sUrl, sOvol)) {
          printf(clientId, "#CLI.LISTNUM#: %*d: %s, %s\n", 3, c, sName, sUrl);
          c++;
        }
      }
      printf(clientId, "##CLI.LIST#\n");
      printf(clientId, "> ");
      return;
    }
    if (strcmp(str, "cli.info") == 0 || strcmp(str, "info") == 0) {
      printf(clientId, "##CLI.INFO#\n");
      char timeStringBuff[50];
      strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S", &network.timeinfo);
      if (config.store.tzHour < 0) {
        printf(clientId, "##SYS.DATE#: %s%03d:%02d\n", timeStringBuff, config.store.tzHour, config.store.tzMin);
      } else {
        printf(clientId, "##SYS.DATE#: %s+%02d:%02d\n", timeStringBuff, config.store.tzHour, config.store.tzMin);
      }
      printf(clientId, "##CLI.NAMESET#: %d %s\n", config.lastStation(), config.station.name);
      if (player.status() == PLAYING) {
        printf(clientId, "##CLI.META#: %s\n", config.station.title);
      }
      printf(clientId, "##CLI.VOL#: %d\n", config.store.volume);
      if (player.status() == PLAYING) {
        printf(clientId, "##CLI.PLAYING#\n");
      } else {
        printf(clientId, "##CLI.STOPPED#\n");
      }
      printf(clientId, "> ");
      return;
    }
    int sb;
    if (sscanf(str, "play(%d)", &sb) == 1 || sscanf(str, "cli.play(\"%d\")", &sb) == 1 || sscanf(str, "play %d", &sb) == 1 ) {
      if (sb < 1) sb = 1;
      if (sb >= config.store.countStation) sb = config.store.countStation;
      player.sendCommand({PR_PLAY, (uint16_t)sb});
      return;
    }
    #ifdef USE_SD
    int mm;
    if (sscanf(str, "mode %d", &mm) == 1 ) {
      if (mm > 2) mm = 0;
      if(mm==2)
        config.changeMode();
      else
        config.changeMode(mm);
      return;
    }
    #endif
    if (strcmp(str, "sys.tzo") == 0 || strcmp(str, "tzo") == 0) {
      printf(clientId, "##SYS.TZO#: %d:%d\n> ", config.store.tzHour, config.store.tzMin);
      return;
    }
    //int16_t tzh, tzm;
    int tzh, tzm;
    if (sscanf(str, "tzo(%d:%d)", &tzh, &tzm) == 2 || sscanf(str, "sys.tzo(\"%d:%d\")", &tzh, &tzm) == 2 || sscanf(str, "tzo %d:%d", &tzh, &tzm) == 2) {
      if (tzh < -12) tzh = -12;
      if (tzh > 14) tzh = 14;
      if (tzm < 0) tzm = 0;
      if (tzm > 59) tzm = 59;
      config.setTimezone((int8_t)tzh, (int8_t)tzm);
      if(tzh<0){
        printf(clientId, "new timezone offset: %03d:%02d\n", config.store.tzHour, config.store.tzMin);
      }else{
        printf(clientId, "new timezone offset: %02d:%02d\n", config.store.tzHour, config.store.tzMin);
      }
      network.requestTimeSync(true);
      return;
    }
    if (sscanf(str, "tzo(%d)", &tzh) == 1 || sscanf(str, "sys.tzo(\"%d\")", &tzh) == 1 || sscanf(str, "tzo %d", &tzh) == 1) {
      if (tzh < -12) tzh = -12;
      if (tzh > 14) tzh = 14;
      config.setTimezone((int8_t)tzh, 0);
      if(tzh<0){
        printf(clientId, "new timezone offset: %03d:%02d\n", config.store.tzHour, config.store.tzMin);
      }else{
        printf(clientId, "new timezone offset: %02d:%02d\n", config.store.tzHour, config.store.tzMin);
      }
      network.requestTimeSync(true);
      return;
    }
    if (sscanf(str, "dspon(%d)", &tzh) == 1 || sscanf(str, "cli.dspon(\"%d\")", &tzh) == 1 || sscanf(str, "dspon %d", &tzh) == 1) {
      config.setDspOn(tzh!=0);
      return;
    }
    if (sscanf(str, "dim(%d)", &tzh) == 1 || sscanf(str, "cli.dim(\"%d\")", &tzh) == 1 || sscanf(str, "dim %d", &tzh) == 1) {
      if (tzh < 0) tzh = 0;
      if (tzh > 100) tzh = 100;
      config.store.brightness = (uint8_t)tzh;
      config.setBrightness(true);
      return;
    }
    if (sscanf(str, "sleep(%d,%d)", &tzh, &tzm) == 2 || sscanf(str, "cli.sleep(\"%d\",\"%d\")", &tzh, &tzm) == 2 || sscanf(str, "sleep %d %d", &tzh, &tzm) == 2) {
      if(tzh>0 && tzm>0) {
        printf(clientId, "sleep for %d minutes after %d minutes ...\n> ", tzh, tzm);
        config.sleepForAfter(tzh, tzm);
      }else{
        printf(clientId, "##CMD_ERROR#\tunknown command <%s>\n> ", str);
      }
      return;
    }
    if (sscanf(str, "sleep(%d)", &tzh) == 1 || sscanf(str, "cli.sleep(\"%d\")", &tzh) == 1 || sscanf(str, "sleep %d", &tzh) == 1) {
      if(tzh>0) {
        printf(clientId, "sleep for %d minutes ...\n> ", tzh);
        config.sleepForAfter(tzh);
      }else{
        printf(clientId, "##CMD_ERROR#\tunknown command <%s>\n> ", str);
      }
      return;
    }
  }
  if (strcmp(str, "sys.version") == 0 || strcmp(str, "version") == 0) {
    printf(clientId, "##SYS.VERSION#: %s\n> ", YOVERSION);
    return;
  }
  if (strcmp(str, "sys.boot") == 0 || strcmp(str, "boot") == 0 || strcmp(str, "reboot") == 0) {
    Player::prepareForRestart();
    ESP.restart();
    return;
  }
  if (strcmp(str, "sys.reset") == 0 || strcmp(str, "reset") == 0) {
    config.reset();
    return;
  }
  if (strcmp(str, "wifi.list") == 0 || strcmp(str, "wifi") == 0) {
    printf(clientId, "#WIFI.SCAN#\n");
    int n = WiFi.scanNetworks();
    if (n == 0) {
        printf(clientId, "no networks found\n");
    } else {
      for (int i = 0; i < n; ++i) {
        printf(clientId, "%d", i + 1);
        printf(clientId, ": ");
        printf(clientId, "%s", WiFi.SSID(i));
        printf(clientId, " (");
        printf(clientId, "%d", WiFi.RSSI(i));
        printf(clientId, ")");
        printf(clientId, (WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
        printf(clientId, "\n");
        delay(10);
      }
    }
    printf(clientId, "#WIFI.SCAN#\n> ");
    return;
  }
  if (strcmp(str, "wifi.con") == 0 || strcmp(str, "conn") == 0) {
    printf(clientId, "#WIFI.CON#\n");
    File file = SPIFFS.open(SSIDS_PATH, "r");
    if (file && !file.isDirectory()) {
      char sSid[BUFLEN], sPas[BUFLEN];
      uint8_t c = 1;
      while (file.available()) {
        if (config.parseSsid(file.readStringUntil('\n').c_str(), sSid, sPas)) {
          printf(clientId, "%d: %s, %s\n", c, sSid, sPas);
          c++;
        }
      }
    }
    printf(clientId, "##WIFI.CON#\n> ");
    return;
  }
  if (strcmp(str, "wifi.station") == 0 || strcmp(str, "station") == 0 || strcmp(str, "ssid") == 0) {
    printf(clientId, "#WIFI.STATION#\n");
    File file = SPIFFS.open(SSIDS_PATH, "r");
    if (file && !file.isDirectory()) {
      char sSid[BUFLEN], sPas[BUFLEN];
      uint8_t c = 1;
      while (file.available()) {
        if (config.parseSsid(file.readStringUntil('\n').c_str(), sSid, sPas)) {
          if(c==config.store.lastSSID) printf(clientId, "%d: %s, %s\n", c, sSid, sPas);
          c++;
        }
      }
    }
    printf(clientId, "##WIFI.STATION#\n> ");
    return;
  }
  char newssid[30], newpass[40];
  if (sscanf(str, "wifi.con(\"%[^\"]\",\"%[^\"]\")", newssid, newpass) == 2 || sscanf(str, "wifi.con(%[^,],%[^)])", newssid, newpass) == 2 || sscanf(str, "wifi.con(%[^ ] %[^)])", newssid, newpass) == 2 || sscanf(str, "wifi %[^ ] %s", newssid, newpass) == 2) {
    char buf[BUFLEN];
    snprintf(buf, BUFLEN, "New SSID: \"%s\" with PASS: \"%s\" for next boot\n> ", newssid, newpass);
    printf(clientId, buf);
    printf(clientId, "...REBOOTING...\n> ");
    memset(buf, 0, BUFLEN);
    snprintf(buf, BUFLEN, "%s\t%s", newssid, newpass);
    config.saveWifiFromNextion(buf);
    return;
  }
  if (strcmp(str, "wifi.status") == 0 || strcmp(str, "status") == 0) {
    printf(clientId, "#WIFI.STATUS#\nStatus:\t\t%d\nMode:\t\t%s\nIP:\t\t%s\nMask:\t\t%s\nGateway:\t%s\nRSSI:\t\t%d dBm\n##WIFI.STATUS#\n> ", 
      WiFi.status(), WiFi.getMode()==WIFI_STA?"WIFI_STA":"WIFI_AP", 
      WiFi.getMode()==WIFI_STA?WiFi.localIP().toString():WiFi.softAPIP().toString(),
      WiFi.getMode()==WIFI_STA?WiFi.subnetMask().toString():"255.255.255.0",
      WiFi.getMode()==WIFI_STA?WiFi.gatewayIP().toString():WiFi.softAPIP().toString(),
      WiFi.RSSI()
    );
    return;
  }
  if (strcmp(str, "wifi.rssi") == 0 || strcmp(str, "rssi") == 0) {
    printf(clientId, "#WIFI.RSSI#\t%d dBm\n> ", WiFi.RSSI());
    return;
  }
  if (strcmp(str, "sys.heap") == 0 || strcmp(str, "heap") == 0) {
    printf(clientId, "Free heap:\t%d bytes\n> ", xPortGetFreeHeapSize());
    return;
  }
  if (strcmp(str, "wifi.discon") == 0 || strcmp(str, "discon") == 0 || strcmp(str, "disconnect") == 0) {
    printf(clientId, "#WIFI.DISCON#\tdisconnected...\n> ");
    WiFi.disconnect();
    return;
  }

#if ES8388_ENABLE
  uint8_t src, vol;
  int svol;
  extern ES8388 es; // single shared instance, owned by player.cpp
  es8388_t &E = config.store.es8388;
  // NOTE: the specific esvolN / eschNbal forms must be tested BEFORE the
  // generic "esvol", because sscanf("esvol %d") also matches "esvol1 30".
  if (sscanf(str, "esvol1 %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 33) svol = 33;
    printf(clientId, "#ES8388.VOL1# set OUT1 (speaker) volume: %d (0-33) \n> ", svol);
    config.saveValue(&E.es_vol1, (uint8_t)svol);
    player.setEs8388Out(ES8388::ES_OUT1, (uint8_t)svol, E.es_bal1);
      return;
  }
  if (sscanf(str, "esvol2 %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 33) svol = 33;
    printf(clientId, "#ES8388.VOL2# set OUT2 (headphone) volume: %d (0-33) \n> ", svol);
    config.saveValue(&E.es_vol2, (uint8_t)svol);
    player.setEs8388Out(ES8388::ES_OUT2, (uint8_t)svol, E.es_bal2);
      return;
  }
  if (sscanf(str, "esch1bal %d", &svol) == 1) {
    if (svol < -6) svol = -6; if (svol > 6) svol = 6;
    printf(clientId, "#ES8388.BAL1# set OUT1 L/R balance: %d (-6..+6) \n> ", svol);
    config.saveValue(&E.es_bal1, (int8_t)svol);
    player.setEs8388Out(ES8388::ES_OUT1, E.es_vol1, (int8_t)svol);
      return;
  }
  if (sscanf(str, "esch2bal %d", &svol) == 1) {
    if (svol < -6) svol = -6; if (svol > 6) svol = 6;
    printf(clientId, "#ES8388.BAL2# set OUT2 L/R balance: %d (-6..+6) \n> ", svol);
    config.saveValue(&E.es_bal2, (int8_t)svol);
    player.setEs8388Out(ES8388::ES_OUT2, E.es_vol2, (int8_t)svol);
      return;
  }
  if (sscanf(str, "esvol %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 254) svol = 254;
    // This used to write the DAC master register directly, which now belongs to
    // the main-page volume and would be overwritten by the next slider or
    // encoder move. It sets the main-page volume instead, in the same 0..254
    // domain as every other volume source, so the value stays coherent and is
    // visible on the main page. There is no separate master-volume setting any
    // more.
    printf(clientId, "#VOL# set volume: %d (0-254) \n> ", svol);
    player.setVol((uint8_t)svol);
      return;
  }
  if (sscanf(str, "esstereo %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 7) svol = 7;
    printf(clientId, "#ES8388.SE# set stereo widening: %d (0-7) \n> ", svol);
    config.saveValue(&E.es_stereo_eff, (uint8_t)svol);
    es.stereo_eff((uint8_t)svol);
      return;
  }
  if (sscanf(str, "esvpp %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 3) svol = 3;
    printf(clientId, "#ES8388.VPP# set DAC Vpp scale: %d (0=3.5V 1=4.0V 2=3.0V 3=2.5V) \n> ", svol);
    config.saveValue(&E.es_vpp, (uint8_t)svol);
    es.vpp_scale((uint8_t)svol);
      return;
  }
  if (sscanf(str, "esdeemph %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 3) svol = 3;
    printf(clientId, "#ES8388.DEEMPH# set de-emphasis: %d (0=off 1=32k 2=44.1k 3=48k) \n> ", svol);
    config.saveValue(&E.es_deemph, (uint8_t)svol);
    es.deemphasis((uint8_t)svol);
      return;
  }
  if (sscanf(str, "esmicpga %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 8) svol = 8;
    printf(clientId, "#ES8388.MICPGA# set mic preamp gain: %d (0..8 = 0..+24dB) \n> ", svol);
    config.saveValue(&E.es_mic_pga, (uint8_t)svol);
    es.mic_gain((uint8_t)svol);
      return;
  }
  if (sscanf(str, "esmicin %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 2) svol = 2;
    printf(clientId, "#ES8388.MICIN# set mic input: %d (0=LIN1 1=LIN2 2=diff) \n> ", svol);
    config.saveValue(&E.es_mic_sel, (uint8_t)svol);
    es.mic_input((uint8_t)svol);
      return;
  }
  if (strcmp(str, "esmono on") == 0)   { config.saveValue(&E.es_mono,(uint8_t)1); es.mono(true);  printf(clientId, "#ES8388.MONO# on\n> "); return; }
  if (strcmp(str, "esmono off") == 0)  { config.saveValue(&E.es_mono,(uint8_t)0); es.mono(false); printf(clientId, "#ES8388.MONO# off\n> "); return; }
  if (strcmp(str, "esramp on") == 0)   { config.saveValue(&E.es_soft_ramp,(uint8_t)1); es.volume_ramp(E.es_ramp_rate); printf(clientId, "#ES8388.RAMP# on\n> "); return; }
  if (strcmp(str, "esramp off") == 0)  { config.saveValue(&E.es_soft_ramp,(uint8_t)0); es.volume_ramp(0); printf(clientId, "#ES8388.RAMP# off\n> "); return; }
  if (strcmp(str, "esvroi on") == 0)   { config.saveValue(&E.es_vroi,(uint8_t)1); es.output_impedance(true);  printf(clientId, "#ES8388.VROI# 40k\n> "); return; }
  if (strcmp(str, "esvroi off") == 0)  { config.saveValue(&E.es_vroi,(uint8_t)0); es.output_impedance(false); printf(clientId, "#ES8388.VROI# 1.5k\n> "); return; }
  if (strcmp(str, "eslinein off") == 0) { config.saveValue(&E.es_linein,(uint8_t)ES8388::LINEIN_OFF);  player.applyOutputRouting(); printf(clientId, "#ES8388.LINEIN# off (radio only)\n> "); return; }
  if (strcmp(str, "eslinein mix") == 0) { config.saveValue(&E.es_linein,(uint8_t)ES8388::LINEIN_MIX);  player.applyOutputRouting(); printf(clientId, "#ES8388.LINEIN# mixed with radio at %d dB\n> ", (int)E.es_linein_gain); return; }
  if (strcmp(str, "eslinein line") == 0){ config.saveValue(&E.es_linein,(uint8_t)ES8388::LINEIN_ONLY); player.applyOutputRouting(); printf(clientId, "#ES8388.LINEIN# line only (radio muted, DAC path off)\n> "); return; }
  if (strcmp(str, "esadc on") == 0)    { config.saveValue(&E.es_adc,(uint8_t)1); es.adc_power(true);  printf(clientId, "#ES8388.ADC# powered up\n> "); return; }
  if (strcmp(str, "esadc off") == 0)   { config.saveValue(&E.es_adc,(uint8_t)0); es.adc_power(false); printf(clientId, "#ES8388.ADC# powered down\n> "); return; }
  if (strcmp(str, "esmicbias on") == 0){ config.saveValue(&E.es_mic_bias,(uint8_t)1); es.mic_bias(true);  printf(clientId, "#ES8388.MICBIAS# on\n> "); return; }
  if (strcmp(str, "esmicbias off") == 0){config.saveValue(&E.es_mic_bias,(uint8_t)0); es.mic_bias(false); printf(clientId, "#ES8388.MICBIAS# off\n> "); return; }
  if (sscanf(str, "esramprate %d", &svol) == 1) {
    if (svol < 0) svol = 0; if (svol > 3) svol = 3;
    printf(clientId, "#ES8388.RAMPRATE# set soft-ramp rate: %d (0-3) \n> ", svol);
    config.saveValue(&E.es_ramp_rate, (uint8_t)svol);
    es.volume_ramp(E.es_soft_ramp ? (uint8_t)svol : 0);
      return;
  }
  if (strcmp(str, "esclick on") == 0) { config.saveValue(&E.es_clickfree,(uint8_t)1); es.click_free(true);  printf(clientId, "#ES8388.CLICKFREE# on\n> "); return; }
  if (strcmp(str, "esclick off") == 0){ config.saveValue(&E.es_clickfree,(uint8_t)0); es.click_free(false); printf(clientId, "#ES8388.CLICKFREE# off\n> "); return; }
  if (strcmp(str, "esinvl on") == 0)  { config.saveValue(&E.es_invl,(uint8_t)1); es.channel_invert(true, E.es_invr);  printf(clientId, "#ES8388.INVL# invert L\n> "); return; }
  if (strcmp(str, "esinvl off") == 0) { config.saveValue(&E.es_invl,(uint8_t)0); es.channel_invert(false, E.es_invr); printf(clientId, "#ES8388.INVL# normal\n> "); return; }
  if (strcmp(str, "esinvr on") == 0)  { config.saveValue(&E.es_invr,(uint8_t)1); es.channel_invert(E.es_invl, true);  printf(clientId, "#ES8388.INVR# invert R\n> "); return; }
  if (strcmp(str, "esinvr off") == 0) { config.saveValue(&E.es_invr,(uint8_t)0); es.channel_invert(E.es_invl, false); printf(clientId, "#ES8388.INVR# normal\n> "); return; }
  if (sscanf(str, "eslingain %d", &svol) == 1) {
    if (svol < -15) svol = -15; if (svol > 6) svol = 6;
    // Snap to the register's 3 dB grid so the stored value is what actually
    // reaches the chip, not something the slider would have to round.
    svol = 6 - ((6 - svol) / 3) * 3;
    printf(clientId, "#ES8388.LINGAIN# set line-in gain: %d dB (-15..+6, 3dB steps) \n> ", svol);
    config.saveValue(&E.es_linein_gain, (int8_t)svol);
    player.applyOutputRouting();
      return;
  }
  if (strcmp(str, "esstandby on") == 0) { config.saveValue(&E.es_standby,(uint8_t)1); player.setEs8388Standby(true);  printf(clientId, "#ES8388.STANDBY# on (codec sleeps when stopped)\n> "); return; }
  if (strcmp(str, "esstandby off") == 0){ config.saveValue(&E.es_standby,(uint8_t)0); player.setEs8388Standby(false); printf(clientId, "#ES8388.STANDBY# off (codec stays awake)\n> "); return; }
  if (strcmp(str, "esspk mute") == 0)  { config.setSpeakerMute(true);  player.setSpeakerMute(true);  printf(clientId, "#ES8388.SPK# muted\n> "); return; }
  if (strcmp(str, "esspk unmute") == 0){ config.setSpeakerMute(false); player.setSpeakerMute(false); printf(clientId, "#ES8388.SPK# unmuted\n> "); return; }
  if (sscanf(str, "esmute2 %d", &svol) == 1) {
    // Saved, then pushed through applyOutputRouting() rather than es.mute(), so
    // the jack-detect override stays the arbiter of what reaches the chip.
    config.saveValue(&E.es_mute2, (uint8_t)(svol!=0));
    player.applyOutputRouting();
    printf(clientId, "#ES8388.MUTE2# headphone amp mute stored: %d%s\n> ",
           svol!=0 ? 1 : 0,
#if HP_DETECT!=255
           player.headphoneForcedMute() ? " (overridden now: no headphone detected)" : ""
#else
           ""
#endif
    );
    return;
  }
  /* Report the jack, so HP_DETECT_ACTIVE can be settled by looking rather than
     guessing: plug a headphone in, run `hp` again, and if "attached" does not
     follow, the level is the other way round and HP_DETECT_ACTIVE should be
     flipped in myoptions.h. */
  if (strcmp(str, "hp") == 0) {
#if HP_DETECT!=255
    int raw = digitalRead(HP_DETECT);
    printf(clientId, "#HP# pin %d raw=%d active=%s | attached=%d forced_mute=%d | es_mute2 stored=%d\n> ",
           HP_DETECT, raw, HP_DETECT_ACTIVE == HIGH ? "HIGH" : "LOW",
           player.headphoneAttached() ? 1 : 0, player.headphoneForcedMute() ? 1 : 0,
           (int)E.es_mute2);
#else
    printf(clientId, "#HP# HP_DETECT is 255 - jack sense not fitted\n> ");
#endif
    return;
  }
  if (strcmp(str, "esreset") == 0) {
      config.setEs8388Defaults();
      player.applyEs8388Settings();
      printf(clientId, "#ES8388.RESET# settings restored to defaults\n> ");
      return;
  }
  if (sscanf(str, "esregw %d %d", &src, &vol) == 2) {
    printf(clientId, "#ES8388.REGW# Write register: %d value: %d\n> ", src, vol);
    es.write_reg(ES8388_ADDR, src, vol);
      return;
  }    
  if (sscanf(str, "esregr %d", &src) == 1) {
    uint8_t v;
    if (es.read_reg(ES8388_ADDR, src, v))
      printf(clientId, "#ES8388.REGR# reg %d = 0x%02X (%d)\n> ", src, v, v);
    else
      printf(clientId, "#ES8388.REGR# read failed\n> ");
      return;
  }
  if (strcmp(str, "esdump") == 0 ) {
      for(int i=0; i<64; i++) {
          uint8_t v = 0;
          es.read_reg(ES8388_ADDR, i, v);
          printf(clientId, "Read REG: %2d    VAL: %3d     bits: ", i, v);
          printf(clientId,PRINTF_BINARY_PATTERN_INT8 "\n", PRINTF_BYTE_TO_BINARY_INT8(v));
      }
      printf(clientId,"\n\n>");
      return;
  }    
      
#endif // ES8388_ENABLE

#ifdef MQTT_ROOT_TOPIC
  // Match a "<command> " prefix and skip past it. The length comes from the
  // literal via sizeof-1 so it is always exactly the literal's length: passing a
  // hand-counted length instead compares the literal's NUL terminator against the
  // first character of the argument and the match silently fails.
  #define TELNET_CMD(lit) (strncmp(str, lit, sizeof(lit) - 1) == 0)
  #define TELNET_ARG(lit) (str + sizeof(lit) - 1)
  // svol above is scoped to the ES8388 block and these MQTT commands are not
  // conditional on that codec, so the broker port gets its own variable.
  int mqttPort;

  if (TELNET_CMD("mqtthost ")) {
      const char *h = TELNET_ARG("mqtthost ");
      if (*h == '-' || *h == '\0') {
          // "-" clears the host, which disables MQTT (config.mqttEnabled() is
          // simply "host is non-empty").
          config.saveValue(config.store.mqtt.host, "", MQTT_HOST_LENGTH);
          mqttReconfigure();
          printf(clientId, "#MQTT# broker host cleared - MQTT disabled\n> ");
      } else {
          config.saveValue(config.store.mqtt.host, h, MQTT_HOST_LENGTH);
          mqttReconfigure();
          printf(clientId, "#MQTT# broker host: %s:%d\n> ", config.store.mqtt.host, config.store.mqtt.port);
      }
      return;
  }
  if (sscanf(str, "mqttport %d", &mqttPort) == 1) {
      if (mqttPort < 1) mqttPort = 1;
      if (mqttPort > 65535) mqttPort = 65535;
      config.saveValue(&config.store.mqtt.port, (uint16_t)mqttPort);
      mqttReconfigure();
      printf(clientId, "#MQTT# broker port: %d\n> ", mqttPort);
      return;
  }
  if (TELNET_CMD("mqtttopic ")) {
      config.saveValue(config.store.mqtt.topic, TELNET_ARG("mqtttopic "), MQTT_TOPIC_LENGTH);
      mqttReconfigure();
      printf(clientId, "#MQTT# root topic: %s\n> ", config.store.mqtt.topic);
      return;
  }
  if (TELNET_CMD("mqttuser ")) {
      const char *u = TELNET_ARG("mqttuser ");
      if (*u == '-' || *u == '\0') {
          // "-" clears it, which reconnects anonymously.
          config.saveValue(config.store.mqtt.user, "", MQTT_USER_LENGTH);
          mqttReconfigure();
          printf(clientId, "#MQTT# user cleared - connecting anonymously\n> ");
      } else {
          config.saveValue(config.store.mqtt.user, u, MQTT_USER_LENGTH);
          mqttReconfigure();
          printf(clientId, "#MQTT# user: %s\n> ", config.store.mqtt.user);
      }
      return;
  }
  if (TELNET_CMD("mqttpass ")) {
      const char *pw = TELNET_ARG("mqttpass ");
      if (*pw == '-' || *pw == '\0') {
          config.saveValue(config.store.mqtt.pass, "", MQTT_PASS_LENGTH);
          mqttReconfigure();
          printf(clientId, "#MQTT# password cleared\n> ");
      } else {
          config.saveValue(config.store.mqtt.pass, pw, MQTT_PASS_LENGTH);
          mqttReconfigure();
          // Deliberately not echoed back.
          printf(clientId, "#MQTT# password set (%d chars)\n> ", (int)strlen(config.store.mqtt.pass));
      }
      return;
  }
  if (strcmp(str, "mqttreset") == 0) {
      config.setMqttDefaults();
      mqttReconfigure();
      printf(clientId, "#MQTT# settings restored to defaults: %s:%d %s\n> ",
             config.store.mqtt.host, config.store.mqtt.port, config.store.mqtt.topic);
      return;
  }
  if (strcmp(str, "mqttstatus") == 0) {
      printf(clientId, "#MQTT# %s host=%s port=%d topic=%s user=%s pass=%s\n> ",
             config.mqttEnabled() ? "enabled" : "disabled (no host)",
             config.store.mqtt.host, config.store.mqtt.port, config.store.mqtt.topic,
             config.store.mqtt.user[0] ? config.store.mqtt.user : "(none)",
             config.store.mqtt.pass[0] ? "(set)" : "(none)");
      return;
  }
#endif //MQTT_ROOT_TOPIC
  #undef TELNET_CMD
  #undef TELNET_ARG
  
  telnet.printf(clientId, "##CMD_ERROR#\tunknown command <%s>\n> ", str);
}
