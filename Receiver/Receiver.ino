// Receiver.ino  —  Car wash vehicle board (ESP32-WROOM-32UE)
//
// Role: receive joystick/button commands from the transmitter over ESP-NOW,
// drive solenoids and the winch/drive motor direction contactors, read the
// four End-of-Travel microswitches, and send EoT + output state back.
//
// SAFETY MODEL
//   - 4 EoT switches are NORMALLY CLOSED. Closed(LOW) = OK, Open(HIGH) = STOP.
//     An open reads the same whether the limit was reached or a wire/switch
//     failed, so any fault fails safe (motor stops).
//   - Motor direction is two 12 V contactors (J10 C/D). They are MUTUALLY
//     EXCLUSIVE: serviceMotor() guarantees both are never energized at once
//     and enforces a dead-time between opposite directions.
//   - All outputs boot OFF and stay OFF until a valid, fresh command arrives.
//   - If the link goes stale (no valid packets within kLinkTimeoutMs), every
//     output is forced to its safe state.
//   - Any EoT trip forces both motor contactors off regardless of command.
//
// WHAT IS VERIFIED vs ASSUMED
//   VERIFIED (from PCB): pin numbers, active-high solenoid gates, four NC EoT
//   switches to GND via internal pull-ups, two direction-contactor outputs.
//   ASSUMED (clearly marked below): which contactor is which physical
//   direction, which EoT limits which direction, dead-time duration, and the
//   button->solenoid mapping. Isolated in applyCommand()/serviceMotor().
//
// Board target: generic ESP32 (ESP32 Dev Module), PSRAM disabled.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>

#include <CapstoneProtocol.h>
#include <BoardPins.h>

// ---------------------------------------------------------------------------
// ESP-NOW receive callback signature changed between arduino-esp32 2.x and
// 3.x. This firmware targets 3.x (const esp_now_recv_info_t*). Fail loudly at
// compile time on an older core rather than producing a wrong-signature build.
// ---------------------------------------------------------------------------
#if !defined(ESP_ARDUINO_VERSION) || \
    ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#error "This firmware requires arduino-esp32 core 3.0.0 or newer."
#endif

using namespace capstone;

// ---------------------------------------------------------------------------
// Link configuration
// ---------------------------------------------------------------------------

// TODO(hardware): set this to the transmitter's STA MAC address. Until then
// the receiver accepts a command from any peer and latches onto the first
// sender (see onCommandReceived). Pin this down before deployment.
static uint8_t g_txMac[6] = {0, 0, 0, 0, 0, 0};
static bool    g_txMacKnown = false;

// ESP-NOW must use the same Wi-Fi channel on both ends.
static const uint8_t kEspNowChannel = 1;

// Link is "stale" if no valid command arrives within this window. Used as the
// general "is the remote there" timeout for entering the safe state.
static const uint32_t kLinkTimeoutMs = 300;

// MOTION HEARTBEAT: the motor is only allowed to run if a fresh, valid command
// arrived within this (much tighter) window. The TX sends at kSendIntervalMs
// (~50 ms / 20 Hz); this allows a couple of dropped packets, then stops the
// motor. For a large, heavy machine this bounds how far it can coast on a lost
// link. Keep it a small multiple of the TX send interval. CONFIG POINT.
static const uint32_t kMotorHeartbeatMs = 150;

// MAX CONTINUOUS RUN: the motor may not run in one direction longer than this
// without the command returning to stop first. Bounds a stuck input or a
// failed EoT from driving indefinitely. Set 0 to disable. CONFIG POINT.
static const uint32_t kMotorMaxRunMs = 20000;  // 20 s

// How often to push feedback back to the transmitter.
static const uint32_t kFeedbackIntervalMs = 100;

// EoT debounce (clear): the switch must read stably CLOSED this long before
// motion is re-enabled, so a flapping/noisy contact does not re-arm the motor.
static const uint32_t kEotClearMs = 25;

// EoT trip confirm (open): an OPEN must persist this long before we trip. A
// short window rejects EMI glitches from the nearby 12 V contactor/motor
// without meaningfully delaying a real end-of-travel stop. Keep this SMALL so
// the fail-safe response stays fast. CONFIG POINT.
static const uint32_t kEotTripConfirmMs = 5;

// ---------------------------------------------------------------------------
// Shared state between the ESP-NOW callback and loop().
// The callback runs in a different task context, so the handoff is guarded by
// a critical section (portMUX). Keep callback work tiny.
// ---------------------------------------------------------------------------

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

struct LinkState {
  volatile bool     hasCommand = false;
  volatile uint32_t lastRxMs   = 0;
  CommandPacket     lastCmd{};
  uint16_t          lastSeq = 0;
  bool              seqInit = false;
};
static LinkState g_link;

// ---------------------------------------------------------------------------
// EoT debounced state — four independent NC microswitches.
// Each switch ties its GPIO to GND; read with INPUT_PULLUP.
//   closed (LOW)  = OK, open (HIGH) = tripped (EoT or fault) -> fails safe.
// ---------------------------------------------------------------------------
struct EotChannel {
  uint8_t  pin;
  bool     tripped      = true;   // assume tripped until proven closed (safe)
  bool     lastRaw      = true;   // last raw sample: true == open
  uint32_t stableSince  = 0;      // when the raw level last changed
};
static EotChannel g_eot[4] = {
  {rx_pins::kEot1},
  {rx_pins::kEot2},
  {rx_pins::kEot3},
  {rx_pins::kEot4},
};

// ---------------------------------------------------------------------------
// Output state cache (for feedback reporting)
// ---------------------------------------------------------------------------
static uint8_t g_outputState = 0;
static bool    g_motorInhibited = false;

// ---------------------------------------------------------------------------
// Motor direction state machine.
//
// J10 C/D drive two 12 V direction contactors for the winch/drive motor. They
// are mutually exclusive: both energized = short / opposing command. To change
// direction we must open the active contactor, wait a dead-time for its
// contacts to physically separate, then close the other. This is done without
// blocking so the safety loop keeps running.
// ---------------------------------------------------------------------------
enum MotorDir : int8_t { kDirStop = 0, kDirC = 1, kDirD = -1 };

// Contactor reversal dead-time: both outputs guaranteed OFF for at least this
// long between OPPOSITE directions, so the contacts physically separate before
// the other closes. Size to the contactor's break time (tens of ms typical);
// err large. CONFIG POINT.
static const uint32_t kMotorDeadTimeMs = 100;

// Motor START dwell: the operator must command the SAME direction continuously
// for at least this long before the contactor actually energizes. This is the
// assistive-tech anti-jitter guard — brief taps, tremor, or a stick flicked
// through a direction will not start the motor. CONFIG POINT; tune to the user.
static const uint32_t kMotorStartDwellMs = 250;

// Waive the start dwell for the FIRST motion right after a fresh arm. Lets the
// operator drive off a limit (or start moving) immediately on re-arming,
// without the dwell delay — useful for a client who should not have to hold
// the stick before it responds. Normal moves still use the full dwell.
// Comment this out to always require the dwell.
#define WAIVE_START_DWELL_ON_FRESH_ARM

struct MotorState {
  int8_t   current      = kDirStop; // what is actually energized right now
  int8_t   desired      = kDirStop; // what the operator is currently asking for
  uint32_t offSince     = 0;        // when both contactors last went OFF
  uint32_t desiredSince = 0;        // when `desired` last changed (for dwell)
  uint32_t runStartMs   = 0;        // when the current energized run began
  bool     maxRunLockout = false;   // true once max-run tripped; clears at stop
  bool     waiting      = false;    // in reversal dead-time before closing
  bool     dwellWaived  = false;    // skip start dwell for the next energize
};
static MotorState g_motor;

// Tracks the transmitter's armed bit across packets so we can detect a fresh
// arm (0 -> 1 transition) on the receiver side.
static bool g_prevArmed = false;

// ---------------------------------------------------------------------------
// Task watchdog. loop() must feed it each pass; if loop() hangs (a blocking
// call, a bug) the chip resets. On reset every output pin floats/releases and
// the hardware gate/contactor pull-downs open everything, so a hang fails to
// the safe state. Critical for an assistive-tech device where a person may be
// in contact with the mechanism. CONFIG POINT (timeout).
// ---------------------------------------------------------------------------
static const uint32_t kWdtTimeoutMs = 1000;

// ===========================================================================
// ESP-NOW receive callback — keep minimal, no serial, no heavy work.
// Signature matches arduino-esp32 3.x (esp_now_recv_info).
// ===========================================================================
void onCommandReceived(const esp_now_recv_info_t* info,
                       const uint8_t* data, int len) {
  if (info == nullptr || data == nullptr || len <= 0) {
    return;
  }

  CommandPacket cmd;
  if (!unpackCommand(data, static_cast<size_t>(len), cmd)) {
    return;  // bad magic/version/type/length/CRC — ignore silently
  }

  // Peer filtering: once we know the transmitter MAC, reject everyone else.
  if (g_txMacKnown) {
    for (int i = 0; i < 6; ++i) {
      if (info->src_addr[i] != g_txMac[i]) {
        return;
      }
    }
  }

  portENTER_CRITICAL_ISR(&g_mux);
  // Drop duplicates / out-of-order using sequence numbers.
  bool accept = !g_link.seqInit || seqIsNewer(cmd.seq, g_link.lastSeq);
  if (accept) {
    g_link.lastSeq   = cmd.seq;
    g_link.seqInit   = true;
    g_link.lastCmd   = cmd;
    g_link.hasCommand = true;
    g_link.lastRxMs  = millis();
    if (!g_txMacKnown) {
      // Latch onto the first valid sender until a MAC is hard-configured.
      for (int i = 0; i < 6; ++i) g_txMac[i] = info->src_addr[i];
      g_txMacKnown = true;
    }
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

// ===========================================================================
// Output helpers — all failsafe-aware.
// ===========================================================================

// Immediately de-energize BOTH direction contactors. LOW = safe. This is the
// only state in which both outputs may share a level. Updates the state
// machine so serviceMotor() observes the dead-time before any re-close.
void motorStop() {
  digitalWrite(rx_pins::kMotorC, LOW);
  digitalWrite(rx_pins::kMotorD, LOW);
  if (g_motor.current != kDirStop) {
    g_motor.offSince = millis();
  }
  g_motor.current = kDirStop;
  g_motor.waiting = false;
  // Returning to stop clears the max-run lockout: the operator must release to
  // stop and then re-command to run again after a max-run trip.
  g_motor.maxRunLockout = false;
}

// Note: g_motor.dwellWaived is intentionally NOT cleared here. It is set on a
// fresh arm and must survive the stop state so the first commanded move after
// arming can skip the dwell. It is cleared only when that move energizes.

// Record the desired direction. Does not touch the outputs directly; the
// interlocked transition happens in serviceMotor(). Call every control cycle.
// Resets the dwell timer whenever the requested direction changes, so the
// start dwell measures continuous demand for one direction.
void requestMotorDir(int8_t dir) {
  if (dir != g_motor.desired) {
    g_motor.desired = dir;
    g_motor.desiredSince = millis();
  }
}

// Nonblocking motor interlock. Guarantees the two contactors are never
// energized at the same time and that a dead-time elapses between opposite
// directions. Call frequently from loop().
void serviceMotor(uint32_t now) {
  // If commanded to stop, or asked to reverse while a contactor is live,
  // open everything first and start the dead-time.
  if (g_motor.desired == kDirStop) {
    if (g_motor.current != kDirStop) motorStop();  // also clears max-run lockout
    return;
  }

  // MAX CONTINUOUS RUN: if a direction has been energized too long, drop it
  // and latch a lockout. The lockout persists (motor stays OFF) until the
  // command returns to stop, so the operator must release before re-running.
  if (g_motor.current != kDirStop && kMotorMaxRunMs != 0 &&
      (now - g_motor.runStartMs) >= kMotorMaxRunMs) {
    motorStop();                    // this clears maxRunLockout...
    g_motor.maxRunLockout = true;   // ...so set it right after
    return;
  }
  if (g_motor.maxRunLockout) {
    return;  // still demanding a direction; stay OFF until released to stop
  }

  if (g_motor.current == g_motor.desired) {
    return;  // already in the right state
  }

  // Need to change to a (different) energized direction.
  if (g_motor.current != kDirStop) {
    // A contactor is live and it's the wrong one: drop both, begin dead-time.
    motorStop();           // sets offSince, current=stop
    g_motor.waiting = true;
    return;
  }

  // current == stop, desired != stop.
  // (a) START DWELL: require continuous demand for this direction before we
  //     energize. Kills jitter / taps / tremor from starting the motor. The
  //     dwell is waived for the first move right after a fresh arm (if the
  //     build enables it).
  if (!g_motor.dwellWaived &&
      (now - g_motor.desiredSince) < kMotorStartDwellMs) {
    return;  // not demanded long enough yet; stay OFF
  }

  // (b) REVERSAL DEAD-TIME: ensure both have been OFF long enough since the
  //     last energized direction before closing a contactor.
  if ((now - g_motor.offSince) < kMotorDeadTimeMs) {
    return;  // still in dead-time; both stay OFF
  }

  g_motor.runStartMs = now;       // start the max-run clock for this run
  g_motor.dwellWaived = false;    // one-shot: next move uses the full dwell
  if (g_motor.desired == kDirC) {
    digitalWrite(rx_pins::kMotorD, LOW);   // belt-and-suspenders
    digitalWrite(rx_pins::kMotorC, HIGH);
    g_motor.current = kDirC;
  } else {  // kDirD
    digitalWrite(rx_pins::kMotorC, LOW);
    digitalWrite(rx_pins::kMotorD, HIGH);
    g_motor.current = kDirD;
  }
  g_motor.waiting = false;
}

void allSolenoidsOff() {
  digitalWrite(rx_pins::kSolMainWater, LOW);
  digitalWrite(rx_pins::kSolSoap,      LOW);
  digitalWrite(rx_pins::kSolSprayer,   LOW);
  digitalWrite(rx_pins::kSolAux,       LOW);
}

void enterSafeState() {
  requestMotorDir(kDirStop);
  motorStop();
  allSolenoidsOff();
  g_motorInhibited = false;  // no command applied; nothing is being inhibited
  g_outputState = 0;
}

// Reads one EoT switch and updates its debounced tripped flag.
// Returns the current (debounced) tripped state.
//
// NC switch to GND with internal pull-up: closed -> LOW, open -> HIGH.
// Open == tripped (end of travel, broken wire, failed switch, unplugged).
// Both transitions are debounced, but asymmetrically:
//   - trip  (open):   confirmed after kEotTripConfirmMs (short, EMI reject)
//   - clear (closed): confirmed after kEotClearMs       (longer, anti-flap)
// Starts tripped, so power-up and any uncertainty fail safe.
bool updateEot(EotChannel& ch, uint32_t now) {
  bool rawOpen = (digitalRead(ch.pin) == HIGH);

  // Restart the stability timer whenever the raw level changes.
  if (rawOpen != ch.lastRaw) {
    ch.lastRaw = rawOpen;
    ch.stableSince = now;
  }

  const uint32_t held = now - ch.stableSince;

  if (rawOpen) {
    // Trip once the open has been stable long enough.
    if (!ch.tripped && held >= kEotTripConfirmMs) {
      ch.tripped = true;
    }
  } else {
    // Clear once the closed contact has been stable long enough.
    if (ch.tripped && held >= kEotClearMs) {
      ch.tripped = false;
    }
  }
  return ch.tripped;
}

// ---------------------------------------------------------------------------
// Command application — THE conservative placeholder mapping lives here.
// Isolated so the motor/solenoid behavior can be tuned without touching the
// link, safety, or feedback logic.
// ---------------------------------------------------------------------------
void applyCommand(const CommandPacket& cmd, bool blockForward,
                  bool blockReverse, bool motorHeartbeatOk) {
  // --- Solenoids ---
  // Only Soap has a transmitter input in the current digital-stick design
  // (top trigger). Mains water / Sprayer / Aux have no remote control yet;
  // they stay OFF here and are available for firmware/automation later.
  // CONFIG POINT: drive kSolMainWater / kSolSprayer / kSolAux from here when
  // their control source is decided (e.g. auto-open Mains while Soap is on).
  const bool wSoap = bitRead(cmd.buttons, kBtnSoap);

  digitalWrite(rx_pins::kSolSoap,      wSoap ? HIGH : LOW);
  digitalWrite(rx_pins::kSolMainWater, LOW);
  digitalWrite(rx_pins::kSolSprayer,   LOW);
  digitalWrite(rx_pins::kSolAux,       LOW);

  bitWrite(g_outputState, kOutSoap,      wSoap);
  bitWrite(g_outputState, kOutMainWater, 0);
  bitWrite(g_outputState, kOutSprayer,   0);
  bitWrite(g_outputState, kOutAux,       0);

  // --- Motor: digital stick directions select a direction contactor. ---
  // Motion rule: any UP => forward (contactor C), any DOWN => reverse
  // (contactor D). Both pressed (fault / EMI, impossible on a real stick) =>
  // STOP, not an arbitrary pick. LEFT and RIGHT are ignored for motion.
  //
  // Motion requires the remote to be ARMED (kTxArmed) and a fresh heartbeat.
  // EoT limits are DIRECTIONAL: a tripped FORWARD limit blocks only forward,
  // a tripped REVERSE limit blocks only reverse, so the operator can always
  // drive off a limit (see blockForward/blockReverse from the loop).
  const bool armed = bitRead(cmd.status, kTxArmed);
  const bool up    = bitRead(cmd.dirs, kDirUp);
  const bool down  = bitRead(cmd.dirs, kDirDown);

#ifdef WAIVE_START_DWELL_ON_FRESH_ARM
  // Fresh arm (disarmed -> armed): waive the start dwell for the next motion so
  // the operator can drive off immediately. Consumed on the next energize.
  if (armed && !g_prevArmed) {
    g_motor.dwellWaived = true;
  }
#endif
  g_prevArmed = armed;

  // Base gate: disarmed or stale heartbeat stops ALL motion.
  const bool baseInhibit = !armed || !motorHeartbeatOk;

  int8_t desired = kDirStop;
  if (!baseInhibit) {
    if (up && !down && !blockForward) {
      desired = kDirC;   // FORWARD (CONFIG POINT: C = physical forward)
    } else if (down && !up && !blockReverse) {
      desired = kDirD;   // REVERSE (CONFIG POINT: D = physical reverse)
    }
    // up&&down, or a direction blocked by its EoT limit -> kDirStop (fail safe)
  }
  requestMotorDir(desired);

  // "Inhibited" for feedback = commanded a direction but it was blocked (by
  // base gate OR by the limit for that direction).
  const bool wantedMotion = (up != down);  // exactly one direction requested
  g_motorInhibited = wantedMotion && (desired == kDirStop);

  // Report intent now; serviceMotor() reconciles the actual outputs.
  bitWrite(g_outputState, kOutMotorActive, desired != kDirStop ? 1 : 0);
  bitWrite(g_outputState, kOutMotorInhibited, g_motorInhibited ? 1 : 0);
}

// ===========================================================================
// Feedback
// ===========================================================================
void sendFeedback(uint16_t ackSeq, uint8_t eotMask) {
  if (!g_txMacKnown) {
    return;  // nobody to talk to yet
  }

  FeedbackPacket fb{};
  fb.seq = ackSeq;
  fb.eot_state = eotMask;  // bits kEot1..kEot4
  fb.output_state = g_outputState;

  uint8_t frame[kFeedbackFrameSize];
  size_t n = packFeedback(fb, frame, sizeof(frame));
  if (n == 0) {
    return;
  }
  esp_now_send(g_txMac, frame, n);
}

// Ensure the transmitter is registered as an ESP-NOW peer before sending.
void ensurePeer() {
  if (!g_txMacKnown) return;
  if (esp_now_is_peer_exist(g_txMac)) return;

  esp_now_peer_info_t peer{};
  for (int i = 0; i < 6; ++i) peer.peer_addr[i] = g_txMac[i];
  peer.channel = kEspNowChannel;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// ===========================================================================
// Setup / loop
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("[rx] boot"));

  // --- Outputs OFF first, before enabling as outputs. ---
  pinMode(rx_pins::kSolMainWater, OUTPUT); digitalWrite(rx_pins::kSolMainWater, LOW);
  pinMode(rx_pins::kSolSoap,      OUTPUT); digitalWrite(rx_pins::kSolSoap,      LOW);
  pinMode(rx_pins::kSolSprayer,   OUTPUT); digitalWrite(rx_pins::kSolSprayer,   LOW);
  pinMode(rx_pins::kSolAux,       OUTPUT); digitalWrite(rx_pins::kSolAux,       LOW);

  pinMode(rx_pins::kMotorC, OUTPUT); digitalWrite(rx_pins::kMotorC, LOW);
  pinMode(rx_pins::kMotorD, OUTPUT); digitalWrite(rx_pins::kMotorD, LOW);

  // --- EoT switches: four NC contacts to GND, internal pull-ups. ---
  for (int i = 0; i < 4; ++i) {
    pinMode(g_eot[i].pin, INPUT_PULLUP);
  }

  enterSafeState();

  // --- Wi-Fi / ESP-NOW ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[rx] esp_now_init FAILED"));
    // Without the link we must stay safe. Halt in safe state.
    while (true) { enterSafeState(); delay(100); }
  }
  esp_now_register_recv_cb(onCommandReceived);

  // --- Task watchdog: reset the chip if loop() stops feeding it. ---
  // On reset all outputs release and the hardware pull-downs open the motor
  // contactors and solenoid MOSFETs -> safe. arduino-esp32 3.x API.
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms = kWdtTimeoutMs,
    .idle_core_mask = 0,   // don't watch idle tasks
    .trigger_panic = true, // panic -> reset on timeout
  };
  // The Arduino core may or may not have already initialized the TWDT
  // (depends on build config). reconfigure() works if it's running; if not,
  // it returns ESP_ERR_INVALID_STATE and we init() instead. Then subscribe
  // the current (loop/Arduino) task.
  if (esp_task_wdt_reconfigure(&wdt_cfg) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_init(&wdt_cfg);
  }
  esp_task_wdt_add(NULL);

  Serial.print(F("[rx] STA MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.println(F("[rx] ready"));
}

void loop() {
  const uint32_t now = millis();

  // --- Snapshot the latest command from the callback safely. ---
  CommandPacket cmd{};
  bool     haveCmd = false;
  uint32_t lastRx  = 0;
  portENTER_CRITICAL(&g_mux);
  haveCmd = g_link.hasCommand;
  cmd     = g_link.lastCmd;
  lastRx  = g_link.lastRxMs;
  portEXIT_CRITICAL(&g_mux);

  // --- EoT first: safety input drives everything. ---
  // Each switch blocks only the direction it limits, so the operator can
  // always drive AWAY from a tripped limit. A broken/unplugged switch reads
  // tripped (NC fail-safe) and blocks its tagged direction.
  uint8_t eotMask = 0;
  bool blockForward = false;
  bool blockReverse = false;
  for (int i = 0; i < 4; ++i) {
    bool tripped = updateEot(g_eot[i], now);
    bitWrite(eotMask, i, tripped ? 1 : 0);  // bit i == kEot1..kEot4
    if (tripped) {
      if (rx_pins::kEotLimits[i] == rx_pins::kLimitsForward) blockForward = true;
      else                                                   blockReverse = true;
    }
  }

  // --- Link freshness (two levels). ---
  // General link timeout -> full safe state (motor + solenoids off).
  const bool linkStale = !haveCmd || (now - lastRx) > kLinkTimeoutMs;
  // Tighter MOTION HEARTBEAT -> motor only runs on a very recent command.
  // Bounds how far a heavy machine can coast if the receiver loses the remote
  // mid-travel, independent of the looser link timeout above.
  const bool motorHeartbeatOk =
      haveCmd && (now - lastRx) <= kMotorHeartbeatMs;

  if (linkStale) {
    // No fresh command at all: everything off.
    enterSafeState();
  } else {
    // Link is alive enough for solenoids/feedback; motor additionally gated by
    // the heartbeat and per-direction EoT limits.
    applyCommand(cmd, blockForward, blockReverse, motorHeartbeatOk);
  }

  // --- Service the interlocked motor contactors every cycle. ---
  // Runs regardless of link state so dead-time and mutual exclusion are always
  // enforced, even while transitioning into the safe state.
  serviceMotor(now);

  // --- Periodic feedback to the transmitter. ---
  static uint32_t lastFb = 0;
  if (now - lastFb >= kFeedbackIntervalMs) {
    lastFb = now;
    ensurePeer();
    uint16_t ackSeq = haveCmd ? cmd.seq : 0;
    sendFeedback(ackSeq, eotMask);
  }

  // --- Feed the watchdog last: a hang anywhere above fails to a reset. ---
  esp_task_wdt_reset();

  // Nonblocking: no delay(). Loop spins fast; work is time-gated above.
}
