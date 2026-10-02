#ifndef i2cbuses_h
#define i2cbuses_h
/* I2C bus ownership.
 *
 * The ESP32 has two I2C controllers. The Arduino core hard-wires the global
 * "Wire" to I2C0 (Wire.cpp: TwoWire Wire = TwoWire(0)) and cannot be
 * re-pointed, so a second bus means using I2C1 through "Wire1" or your own
 * TwoWire(1).
 *
 * On the ESP32-A1S, I2C0 already belongs to the ES8388 codec and cannot be
 * shared without care: the codec is brought up in player.init() with an
 * unbounded retry loop, so anything that claims the bus first with different
 * pins leaves the codec unreachable at 0x10 and the firmware spinning on
 * "Failed!" forever. Peripherals therefore get their own bus, and this header
 * is the single place that opens it.
 *
 * Why the function-local static matters: TwoWire pins can be set exactly once.
 * A second begin() on a running bus returns true and silently keeps the old
 * pins, and setPins() on a running bus returns false. Handing every caller the
 * same already-begun object means there is only ever one begin(), so which
 * peripheral happens to be initialised first no longer decides the pins.
 *
 * When I2C2_SDA/I2C2_SCL are unset this falls back to the primary bus, which is
 * exactly the previous behaviour - nothing here changes unless you opt in.
 */
#include <Arduino.h>
#include <Wire.h>
#include "options.h"

#if (I2C2_SDA != 255) && (I2C2_SCL != 255)
  #define I2C2_ENABLED true
#endif

/* The ES8388 bus. The codec driver uses the global Wire directly, so this is
   mainly for callers that want to talk to a second codec-adjacent device with
   a consistent handle. Deliberately does not call begin(): player.init() owns
   that, and doing it here would be a second begin() on I2C0. */
inline TwoWire &i2cPrimaryBus() {
  return Wire;
}

/* The peripheral bus: I2C1 when configured, otherwise the primary bus.
   Returns a reference, so a caller can hold it as TwoWire & and pass it to any
   library that takes a TwoWire*. */
inline TwoWire &i2cPeripheralBus() {
#ifdef I2C2_ENABLED
  static TwoWire peripheralBus(1);
  static bool begun = false;
  if (!begun) {
    begun = true;
    peripheralBus.begin(I2C2_SDA, I2C2_SCL);
  }
  return peripheralBus;
#else
  return Wire;
#endif
}

#endif // i2cbuses_h