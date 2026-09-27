#include "mqtt.h"

#ifdef MQTT_ROOT_TOPIC
#include "WiFi.h"

#include "telnet.h"
#include "player.h"
#include "config.h"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
char topic[140], status[BUFLEN*3], vol[5], buf[20];
// Set once the client has been pointed at a broker. mqttClient keeps retrying on
// its own timer, so a settings change has to be able to tell "not configured
// yet" from "configured, just not connected".
static bool mqttConfigured = false;

bool mqttEnabled() {
  return config.mqttEnabled();
}

/* Build a "<root>/<leaf>" topic into the shared scratch buffer. Returns false
   if the result would not fit, so a too-long configured topic is ignored rather
   than overflowing. */
static bool mqttMakeTopic(const char *leaf) {
  if (!mqttEnabled()) return false;
  const char *root = config.store.mqtt.topic;
  int n = snprintf(topic, sizeof(topic), "%s%s", root, leaf);
  if (n < 0 || (size_t)n >= sizeof(topic)) {
    topic[0] = '\0';
    return false;
  }
  return true;
}

void connectToMqtt() {
  if (!mqttConfigured || !mqttEnabled()) return;
  mqttClient.connect();
}

void mqttInit() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttReconfigure();
}

/* Apply config.store.mqtt to the client: point it at the broker, set the LWT and
   (re)start connecting. Called at boot and whenever the user edits the settings,
   so it must be safe to call repeatedly. */
void mqttReconfigure() {
  // Drop any existing session first. Without this the old LWT stays registered
  // on the previous broker and the old subscribe is never undone.
  if (mqttConfigured) mqttClient.disconnect(true);
  mqttConfigured = false;

  if (!mqttEnabled()) {
    // Empty host means the user turned MQTT off. Stop the retry loop so we do not
    // spin on a broker that is deliberately not configured.
    if (mqttReconnectTimer) xTimerStop(mqttReconnectTimer, 0);
    return;
  }

  // An empty username means connect anonymously. setCredentials is only called
  // when there is something to send: passing an empty user would otherwise
  // configure an empty username rather than no credentials at all.
  if (strlen(config.store.mqtt.user) > 0) {
    mqttClient.setCredentials(config.store.mqtt.user, config.store.mqtt.pass);
  }
  mqttClient.setServer(config.store.mqtt.host, config.store.mqtt.port);
  if (mqttMakeTopic("connection")) {
    mqttClient.setWill(topic, 0, MQTT_RETAIN_ONLINE, "offline");
  }
  mqttConfigured = true;
  connectToMqtt();
}

void onMqttConnect(bool sessionPresent) {
  if (mqttMakeTopic("command")) mqttClient.subscribe(topic, 2);
  mqttPublishOnline();
  mqttPublishStatus();
  mqttPublishVolume();
  mqttPublishPlaylist();
}

void mqttPublishOnline() {
  if(mqttClient.connected() && mqttMakeTopic("connection")){
    memset(status, 0, BUFLEN*3);
    sprintf(status, "%s", "online");
    mqttClient.publish(topic, 0, MQTT_RETAIN_ONLINE, status);
  }
}

void mqttPublishStatus() {
  if(mqttClient.connected() && mqttMakeTopic("status")){
    memset(status, 0, BUFLEN*3);
    sprintf(status, "{\"status\": %d, \"station\": %d, \"name\": \"%s\", \"title\": \"%s\", \"on\": %d}", player.status()==PLAYING?1:0, config.lastStation(), config.station.name, config.station.title, config.store.dspon);
    mqttClient.publish(topic, 0, MQTT_RETAIN, status);
  }
}

void mqttPublishPlaylist() {
  if(mqttClient.connected() && mqttMakeTopic("playlist")){
    memset(status, 0, BUFLEN*3);
    sprintf(status, "http://%s%s", WiFi.localIP().toString().c_str(), PLAYLIST_PATH);
    mqttClient.publish(topic, 0, MQTT_RETAIN, status);
  }
}

void mqttPublishVolume(){
  if(mqttClient.connected() && mqttMakeTopic("volume")){
    memset(vol, 0, 5);
    sprintf(vol, "%d", config.store.volume);
    mqttClient.publish(topic, 0, MQTT_RETAIN, vol);
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  // Only keep retrying while a broker is actually configured. If the user
  // cleared the host, mqttReconfigure() has already stopped the timer.
  if (mqttConfigured && mqttEnabled() && WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (len == 0) return;
  memset(buf, 0, 20);
  strlcpy(buf, payload, len+1);
  if (strcmp(buf, "prev") == 0) {
    player.prev();
    return;
  }
  if (strcmp(buf, "next") == 0) {
    player.next();
    return;
  }
  if (strcmp(buf, "toggle") == 0) {
    player.toggle();
    return;
  }
  if (strcmp(buf, "stop") == 0) {
    player.sendCommand({PR_STOP, 0});
    //telnet.info();
    return;
  }
  if (strcmp(buf, "start") == 0 || strcmp(buf, "play") == 0) {
    player.sendCommand({PR_PLAY, config.lastStation()});
    return;
  }
  if (strcmp(buf, "boot") == 0 || strcmp(buf, "reboot") == 0) {
    ESP.restart();
    return;
  }
  if (strcmp(buf, "volm") == 0) {
    player.stepVol(false);
    return;
  }
  if (strcmp(buf, "volp") == 0) {
    player.stepVol(true);
    return;
  }
  if (strcmp(buf, "turnoff") == 0) {
    uint8_t sst = config.store.smartstart;
    config.setDspOn(0);
    player.sendCommand({PR_STOP, 0});
    //telnet.info();
    delay(100);
    config.saveValue(&config.store.smartstart, sst);
    return;
  }
  if (strcmp(buf, "turnon") == 0) {
    config.setDspOn(1);
    if (config.store.smartstart == 1) player.sendCommand({PR_PLAY, config.lastStation()});
    return;
  }
  int volume;
  if ( sscanf(buf, "vol %d", &volume) == 1) {
    if (volume < 0) volume = 0;
    if (volume > 254) volume = 254;
    player.setVol(volume);
    return;
  }
  int sb;
  if (sscanf(buf, "play %d", &sb) == 1 ) {
    if (sb < 1) sb = 1;
    if (sb >= config.store.countStation) sb = config.store.countStation;
    player.sendCommand({PR_PLAY, (uint16_t)sb});
    return;
  }
  if (strstr(buf, "http")==buf){
    if(len+1>sizeof(player.burl)) return;
    strlcpy(player.burl, payload, len+1);
    return;
  }
}

#endif // #ifdef MQTT_ROOT_TOPIC
