#ifndef player_h
#define player_h
#include "options.h"

#if I2S_DOUT!=255 || I2S_INTERNAL
  #include "../audioI2S/AudioEx.h"
#else
  #include "../audioVS1053/audioVS1053Ex.h"
#endif

#ifdef ES8388_ENABLE
  #include "../audioES8388/ES8388.h"
#endif

#ifndef MQTT_BURL_SIZE
  #define MQTT_BURL_SIZE  512
#endif

#ifndef PLQ_SEND_DELAY
	#define PLQ_SEND_DELAY portMAX_DELAY
#endif

#define PLERR_LN        64
#define SET_PLAY_ERROR(...) {char buff[512 + 64]; sprintf(buff,__VA_ARGS__); setError(buff);}

enum playerRequestType_e : uint8_t { PR_PLAY = 1, PR_STOP = 2, PR_PREV = 3, PR_NEXT = 4, PR_VOL = 5, PR_CHECKSD = 6, PR_VUTONUS = 7 };
struct playerRequestParams_t
{
  playerRequestType_e type;
  int payload;
};

enum plStatus_e : uint8_t{ PLAYING = 1, STOPPED = 2 };

class Player: public Audio {
  private:
    uint32_t    _volTicks;   /* delayed volume save  */
    bool        _volTimer;   /* delayed volume save  */
    uint32_t    _resumeFilePos;
    plStatus_e  _status;
    char        _plError[PLERR_LN];
  private:
    void _stop(bool alreadyStopped = false);
    void _play(uint16_t stationId);
    void _loadVol(uint8_t volume);
  public:
    bool lockOutput = true;
    bool resumeAfterUrl = false;
    uint32_t sd_min, sd_max;
    #ifdef MQTT_ROOT_TOPIC
    char      burl[MQTT_BURL_SIZE];  /* buffer for browseUrl  */
    #endif
  public:
    Player();
    void init();
    void loop();
    void initHeaders(const char *file);
    void setError(const char *e);
    bool hasError() { return strlen(_plError)>0; }
    void sendCommand(playerRequestParams_t request);
    void resetQueue();
    #ifdef MQTT_ROOT_TOPIC
    void browseUrl();
    #endif
    bool remoteStationName = false;
    plStatus_e status() { return _status; }
    void prev();
    void next();
    void toggle();
    void stepVol(bool up);
    void setVol(uint8_t volume);
    #ifndef ES8388_ENABLE
    /* userVolume (0..254) -> software gain (0..254), folding in the per-station
       ovol trim. ES8388 builds attenuate in the codec instead, so this only
       exists for the software volume path. */
    uint8_t volToI2S(uint8_t volume);
    #endif
    void stopInfo();
    void setOutputPins(bool isPlaying);
    /* User-controlled speaker/amp mute, OR'd into setOutputPins() */
    void setSpeakerMute(bool muted);
    bool speakerMute() const { return _spmute; }
    #ifdef ES8388_ENABLE
    /* Push the persisted ES8388 settings to the codec. */
    void applyEs8388Settings();
    void setEs8388Out(ES8388::ES8388_OUT out, uint8_t vol, int8_t balance);
    /* L/R balance entry point for ES8388 builds. The legacy software balance
       lived in Audio::Gain() and ran as a per-sample multiply over the decoded
       stream; it is gone. balance= (telnet, MQTT, Nextion, Home Assistant and
       the display) now drives the LOUT1 hardware balance instead, so the
       control every integration already uses keeps working. es_bal1 stays the
       single stored value, which is also what the main-page equalizer slider
       writes, so there is one balance per output rather than two that stack.
       balanceToEs8388()/es8388BalanceToLegacy() convert between the legacy
       -16..+16 domain on the wire and the hardware -6..+6. */
    void setEs8388Balance(int8_t legacyBalance);
    int8_t balanceToEs8388(int8_t legacyBalance) const;
    int8_t es8388BalanceToLegacy(int8_t hwBalance) const;
    /* The stored LOUT1 balance, expressed in the legacy domain. */
    int8_t getEs8388Balance() const;
    /* The main-page volume drives the DAC master register (26/27) rather than
       the software multiply, so the whole thing is one 0.5 dB-per-step
       attenuation and both analog outputs follow it. The per-output trims in
       the ES8388 settings then set the speaker/headphone ratio, which the
       master register preserves at every volume. */
    void applyEs8388Volume(uint8_t userVolume);
    /* userVolume (0..254) <-> ES8388::volume() argument (0..192 attenuation).
       ES8388::volume() writes 192 - v to registers 26/27, so 0 is the loudest
       and 192 is -96 dB; user 254 therefore maps to 0 and user 0 to 192. The
       result is a linear-in-dB taper. ovol, the per-station trim from
       playlist.csv, is a dB offset on top. volumeFromEs8388() is the inverse,
       used for stepping. */
    uint8_t volumeToEs8388(uint8_t userVolume) const;
    uint8_t volumeFromEs8388(int atten) const;
    /* Step in register space so a single detent always moves at least one
       0.5 dB step; a step in the 0..254 user domain can round to the same
       register and appear dead. */
    void stepVolumeBy(int steps);
    /* Park/wake the codec with playback; also flips the persisted flag. */
    void setEs8388Standby(bool on);
    /* Mute the amp and silence the codec ahead of a restart. Call this
       immediately before ESP.restart() so the speaker is shut down before the
       CPU stops, instead of floating until the next boot drives the pin. */
    static void prepareForRestart();
    #endif
    void setResumeFilePos(uint32_t pos) { _resumeFilePos = pos; }
  private:
    bool _spmute = false;
#ifdef ES8388_ENABLE
    bool es_standby_wanted = false; // mirror of config.store.es8388.es_standby
    bool es_sleeping = false;       // true while the codec is in standby
#endif
};

extern Player player;

extern __attribute__((weak)) void player_on_start_play();
extern __attribute__((weak)) void player_on_stop_play();
extern __attribute__((weak)) void player_on_track_change();
extern __attribute__((weak)) void player_on_station_change();

#endif
