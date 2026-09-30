#include "options.h"
#include "player.h"
#include "config.h"
#include "telnet.h"
#include "display.h"
#include "sdmanager.h"
#include "netserver.h"

#ifdef ES8388_ENABLE
  #include "../audioES8388/ES8388.h"
  // Single shared codec instance. player.cpp owns it; netserver.cpp and
  // telnet.cpp use this one instead of declaring their own.
  ES8388 es;
#endif // ES8388_ENABLE


Player player;
QueueHandle_t playerQueue;

#if VS1053_CS!=255 && !I2S_INTERNAL
  #if VS_HSPI
    Player::Player(): Audio(VS1053_CS, VS1053_DCS, VS1053_DREQ, &SPI2) {}
  #else
    Player::Player(): Audio(VS1053_CS, VS1053_DCS, VS1053_DREQ, &SPI) {}
  #endif
  void ResetChip(){
    pinMode(VS1053_RST, OUTPUT);
    digitalWrite(VS1053_RST, LOW);
    delay(30);
    digitalWrite(VS1053_RST, HIGH);
    delay(100);
  }
#else
  #if !I2S_INTERNAL
    Player::Player() {}
  #else
    Player::Player(): Audio(true, I2S_DAC_CHANNEL_BOTH_EN)  {}
  #endif
#endif


void Player::init() {
  Serial.print("##[BOOT]#\tplayer.init\t");
  playerQueue=NULL;
  _resumeFilePos = 0;
  playerQueue = xQueueCreate( 5, sizeof( playerRequestParams_t ) );
  setOutputPins(false);
  delay(50);
  memset(_plError, 0, PLERR_LN);
#ifdef MQTT_ROOT_TOPIC
  memset(burl, 0, MQTT_BURL_SIZE);
#endif
  if(MUTE_PIN!=255) pinMode(MUTE_PIN, OUTPUT);
  #if I2S_DOUT!=255
    #if !I2S_INTERNAL
      setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    #endif
  #else
    SPI.begin();
    if(VS1053_RST>0) ResetChip();
    begin();
  #endif

  #ifdef ES8388_ENABLE
  //++++++++ Add for Audio Kit 2.3 A247 with ES8388 codec
  Serial.printf("Connect to ES8388 codec... ");
  // Init I2C control communication with ES8388
  while (not es.begin(ES8388_SDA, ES8388_SCL))
  {
      Serial.printf("Failed!\n");
      delay(1000);
  }
  Serial.printf("OK\n");

  // Apply the persisted runtime settings (seeded from myoptions.h on reset)
  es_sleeping = false;
  applyEs8388Settings();

  //-------- for Audio Kit 2.3 A247 with ES8388 codec
  #endif // ES8388_ENABLE
  
  setBalance(config.store.balance);
  setTone(config.store.bass, config.store.middle, config.store.trebble);
#ifdef ES8388_ENABLE
  // Software volume is pinned to unity: the main-page volume is attenuated in
  // the DAC's master register instead (see applyEs8388Volume), so nothing
  // should scale the samples before they get there.
  setVolume(254);
#else
  setVolume(0);
#endif
  _spmute = config.store.spmute != 0; // restore user speaker-mute
  _status = STOPPED;
  _volTimer=false;
  //randomSeed(analogRead(0));
  #if PLAYER_FORCE_MONO
    forceMono(true);
  #endif
  _loadVol(config.store.volume);
  setConnectionTimeout(1700, 3700);
  Serial.println("done");
}

#ifdef ES8388_ENABLE
/* Set one analog output from a volume plus an L/R balance offset.
   balance -6..+6: positive favours the left channel. */
void Player::setEs8388Out(ES8388::ES8388_OUT out, uint8_t vol, int8_t balance)
{
    uint8_t base = vol > 33 ? 33 : vol;
    int b = balance;
    if (b > 6) b = 6;
    if (b < -6) b = -6;
    int l = base, r = base;
    if (b > 0) l = base + b; else if (b < 0) r = base - b;
    if (l > 33) l = 33;
    if (r > 33) r = 33;
    es.volume_l(out, (uint8_t)l);
    es.volume_r(out, (uint8_t)r);
}

/* Push every persisted ES8388 setting to the chip. Called at boot and after
   any web/telnet change, so runtime edits and the stored state stay in sync. */
void Player::applyEs8388Settings()
{
    es8388_t &e = config.store.es8388;

    // es_standby is cached in es_standby_wanted because setOutputPins() reads it
    // on every play/stop. Re-sync it here so a stored-value change (a settings
    // reset, a config migration) takes effect instead of leaving the cached flag
    // stale. Disabling standby also has to wake the codec now, otherwise it
    // stays parked until the next play.
    if (es_standby_wanted && !e.es_standby) {
        es.wake();
        es_sleeping = false;
    }
    es_standby_wanted = e.es_standby;

    // Volumes and mutes.
    // The master register is deliberately NOT set here: it is driven by the
    // main-page volume (see applyEs8388Volume), so writing e.es_master_vol here
    // would fight it. The field is kept in config_t only so its size and the
    // EEPROM layout stay stable; it is no longer a user setting.
    setEs8388Out(ES8388::ES_OUT1, e.es_vol1, e.es_bal1);
    setEs8388Out(ES8388::ES_OUT2, e.es_vol2, e.es_bal2);
    es.mute(ES8388::ES_MAIN, e.es_mute_main);
    es.mute(ES8388::ES_OUT1, e.es_mute1);
    es.mute(ES8388::ES_OUT2, e.es_mute2);

    // DAC Control 7 (0x1d): stereo enhancement, mono, Vpp scale
    es.stereo_eff(e.es_stereo_eff > 7 ? 7 : e.es_stereo_eff);
    es.mono(e.es_mono);
    es.vpp_scale(e.es_vpp > 3 ? 3 : e.es_vpp);

    // DAC Control 3 (0x19): soft volume ramp. soft_ramp() sets the enable bit
    // as well as the rate; volume_ramp() only touched the rate, which left the
    // ramp permanently on and made this setting cosmetic.
    es.soft_ramp(e.es_soft_ramp != 0, e.es_ramp_rate > 3 ? 3 : e.es_ramp_rate);

    // DAC Control 6 (0x1c): de-emphasis, click free, phase invert
    es.deemphasis(e.es_deemph > 3 ? 3 : e.es_deemph);
    es.click_free(e.es_clickfree);
    es.channel_invert(e.es_invl, e.es_invr);

    // DAC Control 23 (0x2d): output impedance reference
    es.output_impedance(e.es_vroi);

    // Output mixers: line-in contribution
    es.line_in_mix(e.es_linein, e.es_linein_gain);

    // ADC / microphone
    es.mic_gain(e.es_mic_pga > 8 ? 8 : e.es_mic_pga);
    es.mic_input(e.es_mic_sel > 2 ? 2 : e.es_mic_sel);
    es.mic_bias(e.es_mic_bias);
    es.adc_power(e.es_adc);
}

/* user volume (0..254) -> ES8388::volume() argument (0..192).

   ES8388::volume() takes an ATTENUATION and writes 192 - v to registers 26/27,
   so 0 means 0 dB (loudest) and 192 means -96 dB. Its argument therefore counts
   the wrong way round from the register: to make user 254 the loudest we have to
   hand it 0, and user 0 (mute) has to hand it 192.

   That makes the main-page slider a linear-in-dB taper, where the old software
   multiply was linear in amplitude and therefore spent most of its travel
   crowded into the top few dB.

   ovol is the per-station trim from playlist.csv, applied as a dB offset on top
   (positive ovol = louder, i.e. less attenuation). It is a separate offset rather
   than being folded into the mapping scale, so that at ovol 0 every one of the
   193 values still maps back to a distinct user value and a volume-button detent
   can never round onto the value it started from. */
uint8_t Player::volumeToEs8388(uint8_t userVolume) const
{
    int off = (int)lround(-config.station.ovol * 0.5);
    long atten = lround((double)userVolume * 192.0 / 254.0) - off;
    if (atten < 0) atten = 0;
    if (atten > 192) atten = 192;
    return (uint8_t)atten;
}

/* Inverse of volumeToEs8388(), used when stepping so a detent lands on an exact
   value instead of drifting through the 0..254 domain. */
uint8_t Player::volumeFromEs8388(int atten) const
{
    if (atten < 0) atten = 0;
    if (atten > 192) atten = 192;
    int off = (int)lround(-config.station.ovol * 0.5);
    long base = atten + off;
    if (base < 0) base = 0;
    if (base > 192) base = 192;
    long v = lround((double)base * 254.0 / 192.0);
    if (v < 0) v = 0;
    if (v > 254) v = 254;
    return (uint8_t)v;
}

void Player::stepVolumeBy(int steps)
{
    int reg = volumeToEs8388(config.store.volume);
    int target = reg + steps;
    if (target < 0) target = 0;
    if (target > 192) target = 192;
    // If rounding lands back on the current user value, nudge one register
    // further so the step is never silently swallowed.
    uint8_t v = volumeFromEs8388(target);
    if (v == config.store.volume) {
        if (target > reg && target < 192) v = volumeFromEs8388(target + 1);
        else if (target < reg && target > 0) v = volumeFromEs8388(target - 1);
    }
    setVol(v);
}

void Player::applyEs8388SoftRamp()
{
    uint8_t rate = config.store.es8388.es_ramp_rate > 3 ? 3 : config.store.es8388.es_ramp_rate;
    es.soft_ramp(config.store.es8388.es_soft_ramp != 0, rate);
}

/* The main-page volume now attenuates in the DAC instead of in software.
   The soft ramp is forced on here regardless of the stored preference: a step
   on the master register is a real output discontinuity without it, and with it
   the codec ramps 0.5 dB per few LRCK, which is inaudible even when a slider or
   the encoder is moving quickly. */
void Player::applyEs8388Volume(uint8_t userVolume)
{
    uint8_t rate = config.store.es8388.es_ramp_rate > 3 ? 3 : config.store.es8388.es_ramp_rate;
    es.soft_ramp(true, rate);
    es.volume(ES8388::ES_MAIN, volumeToEs8388(userVolume));
}

/* Enable/disable automatic standby. Turning it off wakes the codec at once if
   it is currently parked, so the change is audible without a reboot. */
void Player::setEs8388Standby(bool on)
{
    es_standby_wanted = on;
    if (!on && es_sleeping) {
        es.wake();
        es_sleeping = false;
    }
}
#endif

void Player::sendCommand(playerRequestParams_t request){
  if(playerQueue==NULL) return;
  xQueueSend(playerQueue, &request, PLQ_SEND_DELAY);
}

void Player::resetQueue(){
	if(playerQueue!=NULL) xQueueReset(playerQueue);
}

void Player::stopInfo() {
  config.setSmartStart(0);
  //telnet.info();
  netserver.requestOnChange(MODE, 0);
}

void Player::setError(const char *e){
  strlcpy(_plError, e, PLERR_LN);
  if(hasError()) {
    config.setTitle(_plError);
    telnet.printf("##ERROR#:\t%s\n", e);
  }
}

void Player::_stop(bool alreadyStopped){
  log_i("%s called", __func__);
  if(config.getMode()==PM_SDCARD && !alreadyStopped) config.sdResumePos = player.getFilePos();
  _status = STOPPED;
  setOutputPins(false);
  if(!hasError()) config.setTitle((display.mode()==LOST || display.mode()==UPDATING)?"":const_PlStopped);
  config.station.bitrate = 0;
  config.setBitrateFormat(BF_UNCNOWN);
  #ifdef USE_NEXTION
    nextion.bitrate(config.station.bitrate);
  #endif
  netserver.requestOnChange(BITRATE, 0);
  display.putRequest(DBITRATE);
  display.putRequest(PSTOP);
  setDefaults();
  if(!alreadyStopped) stopSong();
  if(!lockOutput) stopInfo();
  if (player_on_stop_play) player_on_stop_play();
  pm.on_stop_play();
}

void Player::initHeaders(const char *file) {
  if(strlen(file)==0 || true) return; //TODO Read TAGs
  connecttoFS(sdman,file);
  eofHeader = false;
  while(!eofHeader) Audio::loop();
  //netserver.requestOnChange(SDPOS, 0);
  setDefaults();
}

#ifndef PL_QUEUE_TICKS
  #define PL_QUEUE_TICKS 0
#endif
#ifndef PL_QUEUE_TICKS_ST
  #define PL_QUEUE_TICKS_ST 15
#endif
void Player::loop() {
  if(playerQueue==NULL) return;
  playerRequestParams_t requestP;
  if(xQueueReceive(playerQueue, &requestP, isRunning()?PL_QUEUE_TICKS:PL_QUEUE_TICKS_ST)){
    switch (requestP.type){
      case PR_STOP: _stop(); break;
      case PR_PLAY: {
        if (requestP.payload>0) {
          config.setLastStation((uint16_t)requestP.payload);
        }
        _play((uint16_t)abs(requestP.payload)); 
        if (player_on_station_change) player_on_station_change(); 
        pm.on_station_change();
        break;
      }
      case PR_VOL: {
        config.setVolume(requestP.payload);
#ifdef ES8388_ENABLE
        applyEs8388Volume(requestP.payload);
#else
        Audio::setVolume(volToI2S(requestP.payload));
#endif
        break;
      }
      #ifdef USE_SD
      case PR_CHECKSD: {
        if(config.getMode()==PM_SDCARD){
          if(!sdman.cardPresent()){
            sdman.stop();
            config.changeMode(PM_WEB);
          }
        }
        break;
      }
      #endif
      case PR_VUTONUS:
        if(config.vuThreshold>10) config.vuThreshold -=10;
      default: break;
    }
  }
  Audio::loop();
  if(!isRunning() && _status==PLAYING) _stop(true);
  if(_volTimer){
    if((millis()-_volTicks)>3000){
      config.saveVolume();
      _volTimer=false;
    }
  }
#ifdef MQTT_ROOT_TOPIC
  if(strlen(burl)>0){
    browseUrl();
  }
#endif
}

void Player::prepareForRestart() {
  // A restart takes the CPU down mid-playback. Two things make that audible:
  //   1. MUTE_PIN is an output, and on reset every GPIO reverts to input/high-Z.
  //      If the amp enable pin has no external pull, the amplifier floats and
  //      amplifies whatever the codec's output stage is doing - which is why the
  //      burst is loud regardless of the volume setting. Driving the mute level
  //      first, while we still can, closes that window as far as software can.
  //   2. The DAC is mid-stream; power it down cleanly so the output ramps to
  //      zero rather than collapsing.
  // What this CANNOT cover is the gap between esp_restart() and setup() running,
  // which includes the whole bootloader. Only an external pull-down resistor on
  // MUTE_PIN covers that; see the notes in myoptions.h.
  if(MUTE_PIN!=255) {
    pinMode(MUTE_PIN, OUTPUT);
    digitalWrite(MUTE_PIN, MUTE_LOCK ? !MUTE_VAL : MUTE_VAL);
  }
#ifdef ES8388_ENABLE
  // Power the DAC down regardless of the standby setting: a restart is not a
  // normal stop, so the user's es_standby preference should not keep the output
  // stage alive through it.
  es.mute(ES8388::ES_MAIN, true);
  es.mute(ES8388::ES_OUT1, true);
  es.mute(ES8388::ES_OUT2, true);
  es.standby();
#endif
  // Let the amp discharge and the output settle before the pins are released.
  delay(200);
}

void Player::setOutputPins(bool isPlaying) {
  if(REAL_LEDBUILTIN!=255) digitalWrite(REAL_LEDBUILTIN, LED_INVERT?!isPlaying:isPlaying);
  // MUTE_VAL is the level that mutes; the user speaker-mute flag forces it.
  bool ampOn = isPlaying && !_spmute;
  bool _ml = MUTE_LOCK ? !MUTE_VAL : (ampOn ? !MUTE_VAL : MUTE_VAL);
  if(MUTE_PIN!=255) digitalWrite(MUTE_PIN, _ml);
#ifdef ES8388_ENABLE
  // Optionally park the codec in standby while nothing is playing, and wake it
  // again before the first sample of playback. Driven by the persisted
  // es_standby flag so the user can toggle it without reflashing.
  if (es_standby_wanted) {
    if (isPlaying && es_sleeping) es.wake();
    else if (!isPlaying && !es_sleeping) es.standby();
    es_sleeping = !isPlaying;
  }
#endif
}

void Player::setSpeakerMute(bool muted) {
  _spmute = muted;
  setOutputPins(_status == PLAYING);
}

void Player::_play(uint16_t stationId) {
  log_i("%s called, stationId=%d", __func__, stationId);
  setError("");
  remoteStationName = false;
  config.setDspOn(1);
  config.vuThreshold = 0;
  //display.putRequest(PSTOP);
  config.screensaverTicks=SCREENSAVERSTARTUPDELAY;
  config.screensaverPlayingTicks=SCREENSAVERSTARTUPDELAY;
  if(config.getMode()!=PM_SDCARD) {
  	display.putRequest(PSTOP);
  }
  setOutputPins(false);
  //config.setTitle(config.getMode()==PM_WEB?const_PlConnect:"");
  config.setTitle(config.getMode()==PM_WEB?const_PlConnect:"[next track]");
  config.station.bitrate=0;
  config.setBitrateFormat(BF_UNCNOWN);
  config.loadStation(stationId);
  _loadVol(config.store.volume);
  display.putRequest(DBITRATE);
  display.putRequest(NEWSTATION);
  netserver.requestOnChange(STATION, 0);
  netserver.loop();
  //netserver.loop();
  config.setSmartStart(0);
  bool isConnected = false;
  if(config.getMode()==PM_SDCARD && SDC_CS!=255){
    isConnected=connecttoFS(sdman,config.station.url,config.sdResumePos==0?_resumeFilePos:config.sdResumePos-player.sd_min);
  }else {
    config.saveValue(&config.store.play_mode, static_cast<uint8_t>(PM_WEB));
  }
  if(config.getMode()==PM_WEB) isConnected=connecttohost(config.station.url);
  if(isConnected){
  //if (config.store.play_mode==PM_WEB?connecttohost(config.station.url):connecttoFS(SD,config.station.url,config.sdResumePos==0?_resumeFilePos:config.sdResumePos-player.sd_min)) {
    _status = PLAYING;
    if(config.getMode()==PM_SDCARD) {
      config.sdResumePos = 0;
      config.saveValue(&config.store.lastSdStation, stationId);
    }
    //config.setTitle("");
    config.setSmartStart(1);
    netserver.requestOnChange(MODE, 0);
    setOutputPins(true);
    display.putRequest(NEWMODE, PLAYER);
    display.putRequest(PSTART);
    if (player_on_start_play) player_on_start_play();
    pm.on_start_play();
  }else{
    telnet.printf("##ERROR#:\tError connecting to %s\n", config.station.url);
    SET_PLAY_ERROR("Error connecting to %s", config.station.url);
    _stop(true);
  };
}

#ifdef MQTT_ROOT_TOPIC
void Player::browseUrl(){
  setError("");
  remoteStationName = true;
  config.setDspOn(1);
  resumeAfterUrl = _status==PLAYING;
  display.putRequest(PSTOP);
//  setDefaults();
  setOutputPins(false);
  config.setTitle(const_PlConnect);
  if (connecttohost(burl)){
    _status = PLAYING;
    config.setTitle("");
    netserver.requestOnChange(MODE, 0);
    setOutputPins(true);
    display.putRequest(PSTART);
    if (player_on_start_play) player_on_start_play();
    pm.on_start_play();
  }else{
    telnet.printf("##ERROR#:\tError connecting to %s\n", burl);
    SET_PLAY_ERROR("Error connecting to %s", burl);
    _stop(true);
  }
  memset(burl, 0, MQTT_BURL_SIZE);
}
#endif

void Player::prev() {
  
  uint16_t lastStation = config.lastStation();
  if(config.getMode()==PM_WEB || !config.store.sdsnuffle){
    if (lastStation == 1) config.lastStation(config.store.countStation); else config.lastStation(lastStation-1);
  }
  sendCommand({PR_PLAY, config.lastStation()});
}

void Player::next() {
  uint16_t lastStation = config.lastStation();
  if(config.getMode()==PM_WEB || !config.store.sdsnuffle){
    if (lastStation == config.store.countStation) config.lastStation(1); else config.lastStation(lastStation+1);
  }else{
    config.lastStation(random(1, config.store.countStation));
  }
  sendCommand({PR_PLAY, config.lastStation()});
}

void Player::toggle() {
  if (_status == PLAYING) {
    sendCommand({PR_STOP, 0});
  } else {
    sendCommand({PR_PLAY, config.lastStation()});
  }
}

void Player::stepVol(bool up) {
#ifdef ES8388_ENABLE
  // Step in master-register space so a detent is always at least one 0.5 dB
  // step. volsteps keeps its meaning as a multiplier, but now in 0.5 dB units
  // rather than 0..254 user units, so it lands on real register values.
  stepVolumeBy(up ? (int)config.store.volsteps : -(int)config.store.volsteps);
#else
  if (up) {
    if (config.store.volume <= 254 - config.store.volsteps) {
      setVol(config.store.volume + config.store.volsteps);
    }else{
      setVol(254);
    }
  } else {
    if (config.store.volume >= config.store.volsteps) {
      setVol(config.store.volume - config.store.volsteps);
    }else{
      setVol(0);
    }
  }
#endif
}

uint8_t Player::volToI2S(uint8_t volume) {
  int vol = map(volume, 0, 254 - config.station.ovol * 3 , 0, 254);
  if (vol > 254) vol = 254;
  if (vol < 0) vol = 0;
  return vol;
}

void Player::_loadVol(uint8_t volume) {
#ifdef ES8388_ENABLE
  applyEs8388Volume(volume);
#else
  setVolume(volToI2S(volume));
#endif
}

void Player::setVol(uint8_t volume) {
  _volTicks = millis();
  _volTimer = true;
  player.sendCommand({PR_VOL, volume});
}
