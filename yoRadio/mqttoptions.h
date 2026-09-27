#define MQTT_HOST   "192.168.1.57"
#define MQTT_PORT   1883
#define MQTT_USER   ""
#define MQTT_PASS   ""

#define MQTT_ROOT_TOPIC  "yoradio/lab/"

/*
Topics:
MQTT_ROOT_TOPIC/command     // Commands
MQTT_ROOT_TOPIC/status      // Player status
MQTT_ROOT_TOPIC/playlist    // Playlist URL
MQTT_ROOT_TOPIC/volume      // Current volume

Commands:
prev          // prev station
next          // next station
toggle        // start/stop playing
stop          // stop playing
start, play   // start playing
boot, reboot  // reboot
vol x         // set volume
play x        // play station x
*/
