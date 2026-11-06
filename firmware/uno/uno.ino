// Arduino Uno / Nano (ATmega328P) -> ESC throttle (PWM) + serial REPL.
// The Raspberry Pi Pico version lives in ../pico/pico.ino.
//
// Same commands and same status/OK/ERR/WARN/DBG output format as the Pico version, so the
// desktop app (esc_control.py) works with it unchanged. Differences from the Pico version:
//   - ESC signal is on D9 (Servo library, Timer1), not GP17.
//   - Uses the Uno's real EEPROM for save/defaults (no commit() needed).
//   - "reboot" uses the watchdog; "bootsel" doesn't exist on AVR (not needed - upload from the IDE).
//   - RAM is 2 KB, so strings live in flash (F()/PSTR) and there is no String class.
//   - "arm" discards incoming serial for the 3 s hold instead of letting it overflow the 64-byte
//     RX buffer.
//
// Wiring: D9 -> ESC signal, GND -> ESC GND. Do NOT connect the ESC's 5V BEC to the Uno while USB
// is plugged in. ESC power comes from the battery / bench supply, not from USB.
//
// NOTE: the Uno auto-resets every time a program opens its serial port (DTR toggle), and its
// bootloader eats serial data for ~1 s after reset. Wait ~2 s after connecting before sending
// anything (the updated esc_control.py does this automatically for non-Pico boards).

#include <Servo.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

const uint8_t ESC_PIN     = 9;
const int     HARD_MIN_US = 800;    // absolute limits for any pulse we will ever output
const int     HARD_MAX_US = 2200;

// factory defaults (used when nothing is saved in EEPROM)
const int      DEF_MIN_US     = 1000;
const int      DEF_START_US   = 1187;
const int      DEF_MAX_US     = 1830;
const int      DEF_SLEW_PCT   = 1;
const uint32_t DEF_TIMEOUT_MS = 1000;

// runtime-adjustable settings
int      minUs     = DEF_MIN_US;     // 0 % pulse (also the arming pulse)
int      startUs   = DEF_START_US;   // 1 % pulse = lowest pulse where the motor actually spins
int      maxUs     = DEF_MAX_US;     // 100 % pulse = where the motor stops speeding up
int      slewPct   = DEF_SLEW_PCT;   // max change per 20 ms tick, in % of the start..max range
uint32_t timeoutMs = DEF_TIMEOUT_MS; // failsafe: no command/ping for this long -> cut throttle (0 = off)
bool     debugOn   = false;

// Motor direction mode. Normal = unidirectional (0 % = minUs ... 100 % = maxUs). Bidirectional =
// the ESC is set to "bidirectional/3D" in BLHeli_S: the stop pulse is the PPM *centre*, forward
// runs centre..PPM max, reverse runs centre..PPM min. Defaults are BLHeli_S's own defaults.
const int DEF_PPM_MIN = 1148;
const int DEF_PPM_MAX = 1832;
const int DEF_PPM_CTR = 1488;
bool     bidir     = false;
int      dirSign   = 1;              // +1 forward, -1 reverse (bidirectional only)
int      ppmMin    = DEF_PPM_MIN;
int      ppmMax    = DEF_PPM_MAX;
int      ppmCtr    = DEF_PPM_CTR;

Servo    esc;
int      targetUs  = DEF_MIN_US;
int      currentUs = DEF_MIN_US;
bool     tripped   = false;          // failsafe fired; latched until a new throttle command
uint32_t lastPing = 0, lastTick = 0, lastDebug = 0;

char     line[40];
uint8_t  lineLen = 0;
char     buf[220];                   // shared formatting buffer (keeps stack small)

// ---- persistent settings (real EEPROM) ----
struct Settings {
  uint32_t magic;
  int16_t  minUs, startUs, maxUs, slewPct;
  uint32_t timeoutMs;
  int16_t  bidir, ppmMin, ppmMax, ppmCtr;   // appended: older saved records fail the range check
};                                           // below and simply keep the defaults for these fields
const uint32_t SETTINGS_MAGIC = 0xE5C0A001;
uint8_t cfgState = 0;                // 0 = defaults, 1 = flash, 2 = modified

const char *cfgName() {
  return cfgState == 1 ? "flash" : (cfgState == 2 ? "modified" : "defaults");
}

// printf-style output with the format string kept in flash (saves RAM on the Uno)
void sayf(const char *fmtP, ...) {
  va_list ap;
  va_start(ap, fmtP);
  vsnprintf_P(buf, sizeof(buf), fmtP, ap);
  va_end(ap);
  Serial.print(buf);
}
#define SAYF(fmt, ...) sayf(PSTR(fmt), ##__VA_ARGS__)

// ---- throttle mapping (long math: int is only 16 bits here) ----
// The pulse that means "stopped": minUs normally, the PPM centre in bidirectional mode.
int idleUs() { return bidir ? ppmCtr : minUs; }

int pctToUs(int p) {
  if (bidir) {
    if (p <= 0) return ppmCtr;
    if (dirSign > 0) return ppmCtr + (int)((long)(ppmMax - ppmCtr) * p / 100);
    return ppmCtr - (int)((long)(ppmCtr - ppmMin) * p / 100);
  }
  if (p <= 0) return minUs;
  return startUs + (int)((long)(maxUs - startUs) * (p - 1) / 99);
}
int usToPct(int us) {
  if (bidir) {                                    // magnitude, 0..100, regardless of direction
    long span = us >= ppmCtr ? (long)(ppmMax - ppmCtr) : (long)(ppmCtr - ppmMin);
    long d    = us >= ppmCtr ? (long)(us - ppmCtr)     : (long)(ppmCtr - us);
    if (span <= 0) return 0;
    long pc = (d * 100 + span / 2) / span;
    if (pc < 0) pc = 0;
    if (pc > 100) pc = 100;
    return (int)pc;
  }
  if (us < startUs) return 0;
  long span = maxUs - startUs;
  long pct = 1 + ((long)(us - startUs) * 99 + span / 2) / span;
  if (pct < 1) pct = 1;
  if (pct > 100) pct = 100;
  return (int)pct;
}

bool isNumber(const char *s) {
  size_t n = strlen(s);
  if (!n || n > 6) return false;
  for (size_t i = 0; i < n; i++) if (!isdigit((unsigned char)s[i])) return false;
  return true;
}

void loadSettings() {
  Settings st;
  EEPROM.get(0, st);
  bool ok = st.magic == SETTINGS_MAGIC
         && st.minUs >= HARD_MIN_US && st.minUs < st.startUs
         && st.startUs <= st.maxUs - 50 && st.maxUs <= HARD_MAX_US
         && st.slewPct >= 1 && st.slewPct <= 100
         && (st.timeoutMs == 0 || (st.timeoutMs >= 200 && st.timeoutMs <= 60000UL));
  if (!ok) return;                        // nothing saved, or data looks corrupt -> keep defaults
  minUs = st.minUs; startUs = st.startUs; maxUs = st.maxUs;
  slewPct = st.slewPct; timeoutMs = st.timeoutMs;
  bool extOk = (st.bidir == 0 || st.bidir == 1)
            && st.ppmMin >= HARD_MIN_US && st.ppmMin + 50 <= st.ppmCtr
            && st.ppmCtr + 50 <= st.ppmMax && st.ppmMax <= HARD_MAX_US;
  if (extOk) { bidir = st.bidir == 1; ppmMin = st.ppmMin; ppmMax = st.ppmMax; ppmCtr = st.ppmCtr; }
  dirSign = 1;                            // direction never persists - always boot going forward
  targetUs = currentUs = idleUs();
  cfgState = 1;
}

// NB: no function may take/return "Settings" - the Arduino IDE auto-generates prototypes above the struct.
void writeSettings(bool valid) {          // valid=false writes a blank record (= "nothing saved")
  Settings st = { valid ? SETTINGS_MAGIC : 0UL, (int16_t)minUs, (int16_t)startUs, (int16_t)maxUs,
                  (int16_t)slewPct, timeoutMs,
                  (int16_t)(bidir ? 1 : 0), (int16_t)ppmMin, (int16_t)ppmMax, (int16_t)ppmCtr };
  EEPROM.put(0, st);
}

void setTarget(int us) {
  targetUs = constrain(us, HARD_MIN_US, HARD_MAX_US);
  tripped  = false;
  lastPing = millis();
}

void cutNow() {                           // immediate stop, no ramp
  targetUs = currentUs = idleUs();
  esc.writeMicroseconds(idleUs());
}

void printStatus(const char *tagP) {
  char t[14];
  if (timeoutMs) snprintf_P(t, sizeof(t), PSTR("%lums"), (unsigned long)timeoutMs);
  else           strcpy_P(t, PSTR("off"));
  SAYF("%S pulse=%dus (%d%%) target=%dus map: 0%%=%dus 1%%=%dus 100%%=%dus slew=%d%%/20ms timeout=%s failsafe=%s cfg=%s mode=%s dir=%s ppm=%d,%d,%d board=uno\n",
       tagP, currentUs, usToPct(currentUs), targetUs, minUs, startUs, maxUs,
       slewPct, t, tripped ? "TRIPPED" : "ok", cfgName(),
       bidir ? "bidir" : "normal", dirSign > 0 ? "fwd" : "rev", ppmMin, ppmMax, ppmCtr);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  <0-100>        set throttle % (silent, used by the PC app)"));
  Serial.println(F("  s <0-100>      set throttle % (with confirmation)"));
  Serial.println(F("  us <800-2200>  set raw pulse width in microseconds (debug)"));
  Serial.println(F("  stop           cut throttle immediately"));
  Serial.println(F("  status         print current state"));
  Serial.println(F("  dirmode normal|bidir   normal = unidirectional PWM; bidir = ESC set to bidirectional"));
  Serial.println(F("  dir fwd|rev    bidir only: choose the spin direction (throttle drops to centre first)"));
  Serial.println(F("  ppmmin <us>    bidir: full-reverse pulse (default 1148)"));
  Serial.println(F("  ppmmax <us>    bidir: full-forward pulse (default 1832)"));
  Serial.println(F("  ppmctr <us>    bidir: stop/centre pulse (default 1488)"));
  Serial.println(F("  min <us>       set 0% (off) pulse (default 1000)"));
  Serial.println(F("  start <us>     set 1% pulse = where the motor starts (default 1187)"));
  Serial.println(F("  max <us>       set 100% pulse = where speed stops rising (default 1830)"));
  Serial.println(F("  slew <1-100>   ramp limit, % per 20 ms (default 1)"));
  Serial.println(F("  timeout <ms>   failsafe time, 200-60000, 0 = disable (default 1000)"));
  Serial.println(F("  debug on|off   stream status twice a second"));
  Serial.println(F("  save           store min/start/max/slew/timeout in EEPROM"));
  Serial.println(F("  defaults       restore factory values and erase what was saved"));
  Serial.println(F("  arm            hold min pulse for 3 s (use after ESC power-up)"));
  Serial.println(F("  ping           refresh failsafe timer (no reply)"));
  Serial.println(F("  reboot         restart the board (watchdog reset)"));
  Serial.println(F("Note: with failsafe on, a serial terminal must send 'ping' at least every second"));
  Serial.println(F("or throttle is cut. Use 'timeout 0' for hands-on typing tests."));
}

#define IS(x) (strcmp_P(cmd, PSTR(x)) == 0)

void handleLine(char *l) {
  while (*l == ' ') l++;
  size_t n = strlen(l);
  while (n && l[n - 1] == ' ') l[--n] = 0;
  if (!n) return;

  char *cmd = l;
  char *arg = l + n;                      // points at the terminating NUL = empty arg
  char *sp = strchr(l, ' ');
  if (sp) { *sp = 0; arg = sp + 1; while (*arg == ' ') arg++; }
  for (char *p = cmd; *p; p++) *p = tolower((unsigned char)*p);
  for (char *p = arg; *p; p++) *p = tolower((unsigned char)*p);

  if (IS("ping")) { lastPing = millis(); return; }

  if (isNumber(cmd) && !*arg) {           // bare number = silent throttle %
    setTarget(pctToUs(constrain(atoi(cmd), 0, 100)));
    return;
  }

  if (IS("help") || IS("?")) {
    printHelp();
  } else if (IS("s")) {
    if (!isNumber(arg)) { Serial.println(F("ERR usage: s <0-100>")); return; }
    int p = constrain(atoi(arg), 0, 100);
    setTarget(pctToUs(p));
    SAYF("OK throttle %d%% -> %dus\n", p, targetUs);
  } else if (IS("us")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < HARD_MIN_US || v > HARD_MAX_US) {
      SAYF("ERR usage: us <%d-%d>\n", HARD_MIN_US, HARD_MAX_US); return;
    }
    setTarget((int)v);
    SAYF("OK raw pulse target %dus\n", targetUs);
  } else if (IS("dirmode")) {
    if (!strcmp_P(arg, PSTR("normal")))                                   bidir = false;
    else if (!strcmp_P(arg, PSTR("bidir")) || !strcmp_P(arg, PSTR("bidirectional"))) bidir = true;
    else { Serial.println(F("ERR usage: dirmode normal|bidir")); return; }
    dirSign = 1;
    cfgState = 2;
    cutNow();
    SAYF("OK dirmode=%s (throttle cut to %dus)\n", bidir ? "bidir" : "normal", idleUs());
  } else if (IS("dir")) {
    if (!bidir) { Serial.println(F("ERR dir only works in bidirectional mode (dirmode bidir)")); return; }
    if (!strcmp_P(arg, PSTR("fwd")) || !strcmp_P(arg, PSTR("forward")))      dirSign = 1;
    else if (!strcmp_P(arg, PSTR("rev")) || !strcmp_P(arg, PSTR("reverse"))) dirSign = -1;
    else { Serial.println(F("ERR usage: dir fwd|rev")); return; }
    cutNow();                             // never flip direction under load: back to centre first
    SAYF("OK dir=%s (throttle cut to centre)\n", dirSign > 0 ? "fwd" : "rev");
  } else if (IS("ppmmin")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < HARD_MIN_US || v > ppmCtr - 50) {
      SAYF("ERR ppmmin must be %d-%d\n", HARD_MIN_US, ppmCtr - 50); return;
    }
    ppmMin = (int)v; cfgState = 2; cutNow();
    SAYF("OK ppmmin=%dus (throttle cut)\n", ppmMin);
  } else if (IS("ppmmax")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < ppmCtr + 50 || v > HARD_MAX_US) {
      SAYF("ERR ppmmax must be %d-%d\n", ppmCtr + 50, HARD_MAX_US); return;
    }
    ppmMax = (int)v; cfgState = 2; cutNow();
    SAYF("OK ppmmax=%dus (throttle cut)\n", ppmMax);
  } else if (IS("ppmctr")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < ppmMin + 50 || v > ppmMax - 50) {
      SAYF("ERR ppmctr must be %d-%d\n", ppmMin + 50, ppmMax - 50); return;
    }
    ppmCtr = (int)v; cfgState = 2; cutNow();
    SAYF("OK ppmctr=%dus (throttle cut)\n", ppmCtr);
  } else if (IS("stop")) {
    cutNow();
    tripped  = false;
    lastPing = millis();
    Serial.println(F("OK stopped"));
  } else if (IS("status")) {
    printStatus(PSTR("STATUS"));
  } else if (IS("min")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < HARD_MIN_US || v >= startUs) {
      SAYF("ERR min must be %d-%d\n", HARD_MIN_US, startUs - 1); return;
    }
    minUs = (int)v; cfgState = 2; cutNow();
    SAYF("OK min=%dus (throttle cut)\n", minUs);
  } else if (IS("start")) {
    long v = atol(arg);
    if (!isNumber(arg) || v <= minUs || v > maxUs - 50) {
      SAYF("ERR start must be %d-%d\n", minUs + 1, maxUs - 50); return;
    }
    startUs = (int)v; cfgState = 2; cutNow();
    SAYF("OK start=%dus (1%%), throttle cut\n", startUs);
  } else if (IS("max")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < startUs + 50 || v > HARD_MAX_US) {
      SAYF("ERR max must be %d-%d\n", startUs + 50, HARD_MAX_US); return;
    }
    maxUs = (int)v; cfgState = 2; cutNow();
    SAYF("OK max=%dus (throttle cut)\n", maxUs);
  } else if (IS("slew")) {
    long v = atol(arg);
    if (!isNumber(arg) || v < 1 || v > 100) { Serial.println(F("ERR usage: slew <1-100>")); return; }
    slewPct = (int)v; cfgState = 2;
    SAYF("OK slew=%d%%/20ms\n", slewPct);
  } else if (IS("timeout")) {
    long v = atol(arg);
    if (!isNumber(arg) || (v != 0 && (v < 200 || v > 60000L))) {
      Serial.println(F("ERR usage: timeout <0 or 200-60000>")); return;
    }
    timeoutMs = (uint32_t)v; cfgState = 2; lastPing = millis();
    if (v) SAYF("OK timeout=%lums\n", (unsigned long)v);
    else   Serial.println(F("OK timeout=off (failsafe disabled)"));
  } else if (IS("debug")) {
    if (!strcmp_P(arg, PSTR("on")))       debugOn = true;
    else if (!strcmp_P(arg, PSTR("off"))) debugOn = false;
    else { Serial.println(F("ERR usage: debug on|off")); return; }
    SAYF("OK debug %s\n", debugOn ? "on" : "off");
  } else if (IS("save")) {
    cutNow();
    writeSettings(true);
    cfgState = 1;
    SAYF("OK saved to EEPROM: 0%%=%dus 1%%=%dus 100%%=%dus slew=%d timeout=%lu (throttle cut)\n",
         minUs, startUs, maxUs, slewPct, (unsigned long)timeoutMs);
  } else if (IS("defaults")) {
    cutNow();
    minUs = DEF_MIN_US; startUs = DEF_START_US; maxUs = DEF_MAX_US;
    slewPct = DEF_SLEW_PCT; timeoutMs = DEF_TIMEOUT_MS;
    bidir = false; dirSign = 1;
    ppmMin = DEF_PPM_MIN; ppmMax = DEF_PPM_MAX; ppmCtr = DEF_PPM_CTR;
    targetUs = currentUs = idleUs();
    esc.writeMicroseconds(idleUs());
    writeSettings(false);                 // magic = 0 -> treated as "nothing saved"
    cfgState = 0;
    Serial.println(F("OK factory defaults restored, saved settings erased"));
  } else if (IS("arm")) {
    cutNow();
    Serial.println(F("OK arming: holding min pulse for 3 s..."));
    uint32_t t0 = millis();
    while (millis() - t0 < 3000UL) {      // discard input while holding, so the 64-byte RX buffer can't overflow
      while (Serial.available()) Serial.read();
    }
    lineLen = 0;
    lastPing = millis();
    Serial.println(F("OK armed"));
  } else if (IS("reboot")) {
    cutNow();
    Serial.println(F("OK rebooting"));
    Serial.flush();
    delay(50);
    wdt_enable(WDTO_15MS);
    for (;;) {}
  } else if (IS("bootsel")) {
    Serial.println(F("ERR bootsel doesn't exist on the Uno - upload from the Arduino IDE instead"));
  } else {
    SAYF("ERR unknown command '%s' (try: help)\n", cmd);
  }
}

void setup() {
  Serial.begin(115200);
  loadSettings();                         // saved values (if any) replace the defaults - done BEFORE attach,
                                          // because Servo emits a 1500 us pulse from attach() until the first write
  esc.attach(ESC_PIN, HARD_MIN_US, HARD_MAX_US);
  esc.writeMicroseconds(idleUs());        // arm: hold minimum throttle (normal) or centre (bidirectional)
  delay(500);                             // short on purpose: the app's ARM button does the 3 s hold
  lastPing = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = 0; lineLen = 0; handleLine(line); }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }

  if (timeoutMs && !tripped && millis() - lastPing > timeoutMs &&
      (targetUs != idleUs() || currentUs != idleUs())) {
    targetUs = idleUs();
    tripped  = true;
    SAYF("WARN failsafe: nothing received for %lums, throttle cut\n", (unsigned long)timeoutMs);
  }

  if (millis() - lastTick >= 20) {
    lastTick = millis();
    long span = bidir ? (long)max(ppmMax - ppmCtr, ppmCtr - ppmMin) : (long)(maxUs - startUs);
    long step = span * slewPct / 100;
    if (step < 1) step = 1;
    if (!bidir && targetUs >= startUs && currentUs < startUs)        currentUs = startUs;
    else if (!bidir && targetUs == minUs && currentUs <= startUs)    currentUs = minUs;
    else if (currentUs < targetUs) {
      long v = (long)currentUs + step;
      currentUs = (int)(v > targetUs ? targetUs : v);
    } else if (currentUs > targetUs) {
      long v = (long)currentUs - step;
      currentUs = (int)(v < targetUs ? targetUs : v);
    }
    esc.writeMicroseconds(currentUs);
  }

  if (debugOn && millis() - lastDebug >= 500) {
    lastDebug = millis();
    printStatus(PSTR("DBG"));
  }
}
