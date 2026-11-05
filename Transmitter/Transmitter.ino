// Transmitter.ino  —  Car wash handheld remote (ESP32-WROOM-32UE)
//
// Role: read the 8-way DIGITAL joystick (4 direction switches) + 2 top buttons
// (Soap trigger, Arm button), send commands to the receiver over ESP-NOW at a
// fixed cadence, and reflect the receiver's End-of-Travel state on the LEDs.
//
// INPUT MODEL (digital)
//   Stick: 4 switches U/D/L/R; diagonals are two adjacent switches.
//     Motion rule (applied on the receiver): any UP = forward, any DOWN =
//     reverse, both/neither = stop. LEFT/RIGHT are read but unused for now.
//   Top trigger -> Soap solenoid.
//   Top button  -> Arm (latching toggle, see ARM MODEL).
//
// ARM MODEL (assistive-tech: no sustained hold required)
//   - A quick PRESS of the arm button toggles armed on; pressing again
//     disarms. The client does not hold anything to operate.
//   - Motion (stick directions) is only sent to the receiver while ARMED; the
//     receiver also independently requires the kTxArmed status bit.
//   - Auto-disarms (and requires a fresh press to re-arm) on ANY safety
//     condition: EoT trip (from receiver), link loss, boot, or an inactivity
//     timeout (armed but no stick input for kArmInactivityMs).
//   - Soap is allowed whether or not armed (it moves no one); arming only
//     gates motion. CONFIG POINT if you want soap gated too.
//
// LED BEHAVIOR
//   Disarmed : LED1 slow-blinks (invite a press to arm), others off.
//   Armed    : LEDs reflect receiver EoT state (switch i -> LED i, tripped=on).
//   Link lost: all four blink together (EoT state unknown). Also disarms.
//
// Board target: generic ESP32 (ESP32 Dev Module), PSRAM disabled.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <CapstoneProtocol.h>
#include <BoardPins.h>

// ESP-NOW receive callback signature requires arduino-esp32 core 3.x.
#if !defined(ESP_ARDUINO_VERSION) || \
    ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#error "This firmware requires arduino-esp32 core 3.0.0 or newer."
#endif

using namespace capstone;

// ---------------------------------------------------------------------------
// Link configuration
// ---------------------------------------------------------------------------

// TODO(hardware): set to the receiver's STA MAC. Broadcast is used until a
// unicast MAC is configured so bring-up works before MACs are known.
static uint8_t g_rxMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static const uint8_t  kEspNowChannel   = 1;   // must match receiver
static const uint32_t kSendIntervalMs  = 50;  // 20 Hz command cadence
static const uint32_t kLinkTimeoutMs   = 300; // no feedback -> link lost
static const uint32_t kBlinkIntervalMs = 200; // link-lost LED blink rate

// Input conditioning
static const uint32_t kDebounceMs = 20;

// ---------------------------------------------------------------------------
// Arm model
// ---------------------------------------------------------------------------

// Inactivity timeout: if armed but no stick input arrives for this long, auto
// disarm. Set to 0 to disable. CONFIG POINT (default 30 s).
static const uint32_t kArmInactivityMs = 30000;

// ---------------------------------------------------------------------------
// Feedback state from receiver (updated in ESP-NOW callback).
// ---------------------------------------------------------------------------
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_lastFeedbackMs = 0;
static volatile uint8_t  g_eotState = 0;       // last known EoT bits
static volatile bool     g_haveFeedback = false;

// ---------------------------------------------------------------------------
// Arm state (latching toggle).
// ---------------------------------------------------------------------------
static bool     g_armed = false;
static uint32_t g_lastActivityMs = 0;  // last time a stick direction was active

// ---------------------------------------------------------------------------
// Debounced digital input. Pressed = LOW (external pull-ups, active-low).
// ---------------------------------------------------------------------------
struct DigitalIn {
  uint8_t  pin;
  bool     stable    = false;  // debounced pressed state
  bool     lastRaw   = false;
  uint32_t changedAt = 0;
};

// Stick direction switches, in DirBit order (Up, Down, Left, Right).
static DigitalIn g_dirs[4] = {
  {tx_pins::kStickUp},
  {tx_pins::kStickDown},
  {tx_pins::kStickLeft},
  {tx_pins::kStickRight},
};

// Top buttons.
static DigitalIn g_soap{tx_pins::kTopSoap};
static DigitalIn g_arm{tx_pins::kTopArm};

static uint16_t g_seq = 0;

// ===========================================================================
// ESP-NOW receive callback (feedback from receiver). Keep minimal.
// ===========================================================================
void onFeedbackReceived(const esp_now_recv_info_t* info,
                        const uint8_t* data, int len) {
  if (data == nullptr || len <= 0) return;

  FeedbackPacket fb;
  if (!unpackFeedback(data, static_cast<size_t>(len), fb)) {
    return;  // invalid frame
  }

  portENTER_CRITICAL_ISR(&g_mux);
  g_eotState       = fb.eot_state;
  g_lastFeedbackMs = millis();
  g_haveFeedback   = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

// ===========================================================================
// Input helpers
// ===========================================================================

// Debounce one input. Returns debounced pressed state. Pressed = LOW.
bool updateInput(DigitalIn& d, uint32_t now) {
  bool rawPressed = (digitalRead(d.pin) == LOW);
  if (rawPressed != d.lastRaw) {
    d.lastRaw = rawPressed;
    d.changedAt = now;
  }
  if ((now - d.changedAt) >= kDebounceMs) {
    d.stable = rawPressed;
  }
  return d.stable;
}

// Rising-edge detector for the arm button (toggle on press, not on release).
bool armPressedEdge(uint32_t now) {
  static bool prev = false;
  bool pressed = updateInput(g_arm, now);
  bool edge = pressed && !prev;
  prev = pressed;
  return edge;
}

// ===========================================================================
// LED update from arm state + receiver feedback + link state.
// ===========================================================================
void updateLeds(uint32_t now, bool linkLost) {
  uint8_t eot;
  portENTER_CRITICAL(&g_mux);
  eot = g_eotState;
  portEXIT_CRITICAL(&g_mux);

  static const uint8_t kEotLeds[4] = {
    tx_pins::kLed1, tx_pins::kLed2, tx_pins::kLed3, tx_pins::kLed4,
  };

  if (!g_armed) {
    // Disarmed: slow-blink LED1 to invite a press; others off.
    bool on = ((now / 500) & 1U) != 0;
    digitalWrite(tx_pins::kLed1, on ? HIGH : LOW);
    digitalWrite(tx_pins::kLed2, LOW);
    digitalWrite(tx_pins::kLed3, LOW);
    digitalWrite(tx_pins::kLed4, LOW);
    return;
  }

  if (linkLost) {
    // Armed but link lost: blink all four (EoT unknown). Note: link loss also
    // disarms in loop(); this covers the same cycle before that takes effect.
    bool on = ((now / kBlinkIntervalMs) & 1U) != 0;
    for (int i = 0; i < 4; ++i) digitalWrite(kEotLeds[i], on ? HIGH : LOW);
    return;
  }

  // Armed, link OK: each LED tracks its EoT switch (tripped = on).
  for (int i = 0; i < 4; ++i) {
    digitalWrite(kEotLeds[i], bitRead(eot, i) ? HIGH : LOW);
  }
}

// ===========================================================================
// Send a command frame.
// ===========================================================================
void sendCommand(uint8_t dirs, uint8_t buttons, bool armed) {
  CommandPacket cmd{};
  cmd.seq     = g_seq++;
  cmd.dirs    = dirs;
  cmd.buttons = buttons;
  cmd.status  = 0;
  bitWrite(cmd.status, kTxArmed, armed ? 1 : 0);

  uint8_t frame[kCommandFrameSize];
  size_t n = packCommand(cmd, frame, sizeof(frame));
  if (n == 0) return;
  esp_now_send(g_rxMac, frame, n);
}

// ===========================================================================
// Setup / loop
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("[tx] boot"));

  // Indicator LEDs OFF first.
  pinMode(tx_pins::kLed1, OUTPUT); digitalWrite(tx_pins::kLed1, LOW);
  pinMode(tx_pins::kLed2, OUTPUT); digitalWrite(tx_pins::kLed2, LOW);
  pinMode(tx_pins::kLed3, OUTPUT); digitalWrite(tx_pins::kLed3, LOW);
  pinMode(tx_pins::kLed4, OUTPUT); digitalWrite(tx_pins::kLed4, LOW);

  // All six inputs: external pull-ups present, idle HIGH / pressed LOW.
  // GPIO34/35 are input-only (no internal pulls) but the board has externals.
  pinMode(tx_pins::kStickUp,    INPUT);
  pinMode(tx_pins::kStickDown,  INPUT);
  pinMode(tx_pins::kStickLeft,  INPUT);
  pinMode(tx_pins::kStickRight, INPUT);
  pinMode(tx_pins::kTopSoap,    INPUT);
  pinMode(tx_pins::kTopArm,     INPUT);

  // Always start disarmed.
  g_armed = false;
  Serial.println(F("[tx] disarmed — press the arm button to enable motion"));

  // Wi-Fi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[tx] esp_now_init FAILED"));
    while (true) {
      // Blink LED4 to signal the fault; send nothing.
      digitalWrite(tx_pins::kLed4, ((millis() / 200) & 1U) ? HIGH : LOW);
      delay(10);
    }
  }
  esp_now_register_recv_cb(onFeedbackReceived);

  // Register the receiver peer (broadcast until a real MAC is set).
  esp_now_peer_info_t peer{};
  for (int i = 0; i < 6; ++i) peer.peer_addr[i] = g_rxMac[i];
  peer.channel = kEspNowChannel;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.print(F("[tx] STA MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.println(F("[tx] ready"));
}

void loop() {
  const uint32_t now = millis();

  // --- Read inputs ---
  uint8_t dirs = 0;
  bool anyDir = false;
  for (int i = 0; i < 4; ++i) {
    bool pressed = updateInput(g_dirs[i], now);
    bitWrite(dirs, i, pressed ? 1 : 0);  // i == kDirUp..kDirRight
    anyDir = anyDir || pressed;
  }
  bool soap = updateInput(g_soap, now);

  // --- Link freshness (also a disarm condition) ---
  bool     haveFb;
  uint32_t lastFb;
  portENTER_CRITICAL(&g_mux);
  haveFb = g_haveFeedback;
  lastFb = g_lastFeedbackMs;
  uint8_t eot = g_eotState;
  portEXIT_CRITICAL(&g_mux);
  const bool linkLost = !haveFb || (now - lastFb) > kLinkTimeoutMs;
  const bool anyEot   = (eot != 0);

  // --- Arm button: latching toggle on rising edge ---
  if (armPressedEdge(now)) {
    g_armed = !g_armed;
    if (g_armed) g_lastActivityMs = now;  // start the inactivity window fresh
    Serial.println(g_armed ? F("[tx] ARMED") : F("[tx] disarmed (button)"));
  }

  // --- Auto-disarm on any safety condition ---
  if (g_armed) {
    if (linkLost) {
      g_armed = false;
      Serial.println(F("[tx] disarmed (link lost)"));
    } else if (anyEot) {
      g_armed = false;
      Serial.println(F("[tx] disarmed (EoT)"));
    } else {
      // Inactivity: any stick direction keeps it alive. (Soap intentionally
      // does NOT count — CONFIG POINT: add `|| soap` to include it.)
      if (anyDir) {
        g_lastActivityMs = now;
      } else if (kArmInactivityMs != 0 &&
                 (now - g_lastActivityMs) >= kArmInactivityMs) {
        g_armed = false;
        Serial.println(F("[tx] disarmed (inactivity)"));
      }
    }
  }

  // --- LEDs ---
  updateLeds(now, linkLost);

  // --- Send at a fixed cadence ---
  static uint32_t lastSend = 0;
  if (now - lastSend >= kSendIntervalMs) {
    lastSend = now;

    // Motion (stick dirs) only when armed. When disarmed, send zero dirs so
    // the receiver holds the motor stopped. Soap is sent regardless of arm.
    uint8_t txDirs = g_armed ? dirs : 0;
    uint8_t buttons = 0;
    bitWrite(buttons, kBtnSoap, soap ? 1 : 0);
    // The arm button's own state rides in the status flag, not as a command.

    sendCommand(txDirs, buttons, g_armed);
  }

  // Nonblocking: no delay() in the main loop.
}
