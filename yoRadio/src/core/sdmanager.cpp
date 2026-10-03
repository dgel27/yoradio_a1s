// options.h has to be visible before the guard below: SDC_CS is a macro, and an
// undefined identifier evaluates to 0 in the preprocessor, so testing it before
// any include made "0 != 255" true and compiled the whole SD stack even when SD
// was not fitted. sdmanager.h includes options.h, so include it first, the same
// way rtcsupport.cpp and touchscreen.cpp do.
#include "sdmanager.h"
#if SDC_CS!=255

#define USE_SD
#include "display.h"
#include "player.h"

#if defined(SD_SPIPINS) || SD_HSPI
SPIClass  SDSPI(HOOPSENb);
#define SDREALSPI SDSPI
#else
  #define SDREALSPI SPI
#endif

#ifndef SDSPISPEED
  #define SDSPISPEED 20000000
#endif

SDManager sdman(FSImplPtr(new VFSImpl()));

/* SDFS::begin() in framework-arduinoespressif32 calls spi.begin() with NO
   arguments, which silently resets the SPI pins to that bus's native set and
   throws away anything SD_SPIPINS asked for. On HSPI (SPI2) those natives are
   12/13/14/15, so MISO landed on GPIO12 - which on the A1S microSD is card pin 1,
   DAT2, explicitly "not used" in SPI mode. The card still initialised, because
   init is write-heavy, but every file read came back empty.

   Pin the pins AFTER SDFS::begin() has done its bare spi.begin(), then call it
   again: the FatFs card object keeps a pointer to the same SPIClass, so it picks
   the new routing up without being rebuilt. Without the second begin() the
   already-mounted card would keep using the old matrix and the fix would look
   like it had done nothing. */
static void pinSdSpiPins()
{
#if defined(SD_SPIPINS) || SD_HSPI
  #if defined(SD_SPIPINS)
    SDREALSPI.begin(SD_SPIPINS);   // SCK, MISO, MOSI - as configured
  #else
    SDREALSPI.begin();             // HSPI natives
  #endif
#endif
}

bool SDManager::start(){
  ready = begin(SDC_CS, SDREALSPI, SDSPISPEED);
  pinSdSpiPins();
  if(ready) {
    // Re-pin, then re-initialise so the card object is built against the right
    // matrix. begin() is a no-op once mounted, so drop the mount first.
    stop();
    ready = begin(SDC_CS, SDREALSPI, SDSPISPEED);
  }
  if(!ready) {
    vTaskDelay(10);
    if(!ready) ready = begin(SDC_CS, SDREALSPI, SDSPISPEED);
    pinSdSpiPins();
    vTaskDelay(10);
    if(!ready) ready = begin(SDC_CS, SDREALSPI, SDSPISPEED);
  }
  return ready;
}

void SDManager::stop(){
  end();
  ready = false;
}
#include "diskio_impl.h"
bool SDManager::cardPresent() {

  if(!ready) return false;
  if(sectorSize()<1) {
    return false;
  }
  uint8_t buff[sectorSize()] = { 0 };
  bool bread = readRAW(buff, 1);
  if(sectorSize()>0 && !bread) return false;
  return bread;
}

bool SDManager::_checkNoMedia(const char* path){
  char nomedia[BUFLEN]= {0};
  strlcat(nomedia, path, BUFLEN);
  strlcat(nomedia, "/.nomedia", BUFLEN);
  bool nm = exists(nomedia);
  return nm;
}

bool SDManager::_endsWith (const char* base, const char* str) {
  int slen = strlen(str) - 1;
  const char *p = base + strlen(base) - 1;
  while(p > base && isspace(*p)) p--;
  p -= slen;
  if (p < base) return false;
  return (strncmp(p, str, slen) == 0);
}

void SDManager::listSD(File &plSDfile, File &plSDindex, const char* dirname, uint8_t levels) {
    File root = sdman.open(dirname);
    if (!root) {
        Serial.println("##[ERROR]#\tFailed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("##[ERROR]#\tNot a directory");
        return;
    }

    uint32_t pos = 0;
    char* filePath;
    while (true) {
        vTaskDelay(2);
        player.loop();
        bool isDir;
        String fileName = root.getNextFileName(&isDir);
        if (fileName.isEmpty()) break;
        filePath = (char*)malloc(fileName.length() + 1);
        if (filePath == NULL) {
            Serial.println("Memory allocation failed");
            break;
        }
        strcpy(filePath, fileName.c_str());
        const char* fn = strrchr(filePath, '/') + 1;
        if (isDir) {
            if (levels && !_checkNoMedia(filePath)) {
                listSD(plSDfile, plSDindex, filePath, levels - 1);
            }
        } else {
            /* Listed extensions must match the ones Audio::connecttoFS()
               actually dispatches on, or the decoders this build enables are
               unreachable from a card: it accepted .ogg/.vorbis/.opus since the
               Vorbis and Opus decoders went in, but only five extensions were
               indexed, so those files were never written to the playlist.

               strlwr() only covers the first test in the original, so a file
               named SONG.MP3 indexed while SONG.FLAC did not. Lowercasing the
               name once up front fixes that and is cheaper than repeating it. */
            strlwr((char*)fn);
            if (_endsWith(fn, ".mp3") || _endsWith(fn, ".m4a") || _endsWith(fn, ".aac") ||
                _endsWith(fn, ".wav") || _endsWith(fn, ".flac") || _endsWith(fn, ".ogg") ||
                _endsWith(fn, ".vorbis") || _endsWith(fn, ".opus")) {
                pos = plSDfile.position();
                plSDfile.printf("%s\t%s\t0\n", fn, filePath);
                plSDindex.write((uint8_t*)&pos, 4);
                Serial.print(".");
                if(display.mode()==SDCHANGE) display.putRequest(SDFILEINDEX, _sdFCount+1);
                _sdFCount++;
                if (_sdFCount % 64 == 0) Serial.println();
            }
        }
        free(filePath);
    }
    root.close();
}

void SDManager::indexSDPlaylist() {
  _sdFCount = 0;
  if(exists(PLAYLIST_SD_PATH)) remove(PLAYLIST_SD_PATH);
  if(exists(INDEX_SD_PATH)) remove(INDEX_SD_PATH);
  File playlist = open(PLAYLIST_SD_PATH, "w", true);
  if (!playlist) {
    return;
  }
  File index = open(INDEX_SD_PATH, "w", true);
  listSD(playlist, index, "/", SD_MAX_LEVELS);
  index.flush();
  index.close();
  playlist.flush();
  playlist.close();
  Serial.println();
  delay(50);
}
#endif


