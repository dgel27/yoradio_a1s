/* MQTT is OFF by default: MQTT_HOST is empty, and an empty broker host is what
 * disables it (Config::mqttEnabled() is simply "host is non-empty"), so nothing
 * tries to connect and no reconnect timer is started.
 *
 * Turn it on without reflashing: Settings -> mqtt in the web UI (host, port,
 * root topic, username, password), or over telnet:
 *
 *     mqtthost 192.168.1.10
 *     mqttport 1883
 *     mqtttopic yoradio/lab/
 *
 * Use "-" as the argument to mqtthost/mqttuser/mqttpass to clear a field;
 * clearing the host switches MQTT off again.
 *
 * THIS FILE MUST EXIST for the firmware to compile. src/core/netserver.cpp
 * references player.burl, which is only declared when MQTT_ROOT_TOPIC is
 * defined. Deleting this file breaks the build, so keep it even if you never
 * use MQTT - an empty MQTT_HOST is the supported "off" configuration.
 */
#define MQTT_HOST   ""            /* broker host; "" = MQTT disabled */
#define MQTT_PORT   1883
#define MQTT_USER   ""
#define MQTT_PASS   ""

#define MQTT_ROOT_TOPIC  "yoradio/"

/*
Topics (each is MQTT_ROOT_TOPIC plus the suffix):
MQTT_ROOT_TOPIC/connection     -> retained "online" / "offline" (last will)
MQTT_ROOT_TOPIC/command        -> subscribe, commands below
MQTT_ROOT_TOPIC/status         -> retained player status JSON
MQTT_ROOT_TOPIC/playlist       -> retained URL of this device's playlist.csv
MQTT_ROOT_TOPIC/volume         -> retained current volume

Commands (payload sent to .../command):
prev          -> previous station
next          -> next station
toggle        -> start/stop playing
stop          -> stop playing
start, play   -> start playing
boot, reboot  -> reboot the device
vol x         -> set volume (0..254)
play x        -> play station x (1-based index)
volm / volp   -> volume down / up
turnon / turnoff
a URL starting with "http" -> play that URL directly
*/
