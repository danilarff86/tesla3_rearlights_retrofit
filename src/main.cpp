#include <Arduino.h>
#include <avr/wdt.h>
#include <stdint.h>

// Pin assignments
constexpr uint8_t kBrakeSignalPin = 2;
constexpr uint8_t kLeftTurnSignalInputPin = 3;
constexpr uint8_t kRightTurnSignalInputPin = 4;
constexpr uint8_t kLeftTurnSignalOutputPin = 5;
constexpr uint8_t kRightTurnSignalOutputPin = 6;

// Input mode for the signal pins. Use INPUT_PULLUP (and invert the logic)
// if the input stage is an open-collector optocoupler.
constexpr uint8_t kSignalInputMode = INPUT;

// Output level meaning "turn signal allowed". This is also the safe state
// the outputs are driven to at boot.
constexpr uint8_t kAllowLevel = LOW;
constexpr uint8_t kSuppressLevel = HIGH;

// Timing, in milliseconds
// A turn signal falling edge keeps the turn signal allowed for this long.
// Must be longer than one full blink period so the output stays allowed
// between flashes.
constexpr uint32_t kTurnSignalWindowMs = 800;
// Turn and brake falling edges closer than this are treated as the same
// event (e.g. both lines dropping when the brake is released).
constexpr uint32_t kEdgeCoincidenceMs = 50;
// An input must hold a new level this long before it is accepted.
constexpr uint32_t kDebounceMs = 20;

// A digital input with debouncing and HIGH -> LOW edge tracking.
struct DebouncedInput {
  uint8_t pin;
  uint8_t stableState;
  uint8_t candidateState;
  uint32_t candidateSinceMs;
  // Time the last accepted HIGH -> LOW transition started.
  uint32_t lastFallingEdgeMs;
  bool hasFallingEdge;

  explicit DebouncedInput(uint8_t inputPin)
      : pin(inputPin), stableState(LOW), candidateState(LOW),
        candidateSinceMs(0), lastFallingEdgeMs(0), hasFallingEdge(false) {}

  void begin(uint32_t nowMs) {
    pinMode(pin, kSignalInputMode);
    stableState = candidateState = digitalRead(pin);
    candidateSinceMs = nowMs;
  }

  void update(uint32_t nowMs) {
    const uint8_t raw = digitalRead(pin);
    if (raw != candidateState) {
      candidateState = raw;
      candidateSinceMs = nowMs;
    } else if (raw != stableState &&
               nowMs - candidateSinceMs >= kDebounceMs) {
      if (stableState == HIGH && raw == LOW) {
        // Timestamp the edge when it started, so the debounce delay does not
        // skew comparisons between inputs.
        lastFallingEdgeMs = candidateSinceMs;
        hasFallingEdge = true;
      }
      stableState = raw;
    }
  }
};

DebouncedInput brakeSignal(kBrakeSignalPin);
DebouncedInput leftTurnSignal(kLeftTurnSignalInputPin);
DebouncedInput rightTurnSignal(kRightTurnSignalInputPin);

// Absolute difference of two millis() timestamps, correct across wrap-around.
uint32_t absDiffMs(uint32_t a, uint32_t b) {
  const uint32_t diff = a - b;
  return static_cast<int32_t>(diff) < 0 ? 0u - diff : diff;
}

bool isTurnSignalAllowed(const DebouncedInput &turnSignal, uint32_t nowMs) {
  if (brakeSignal.stableState == LOW) {
    return true; // Brake signal is not active
  }
  if (!turnSignal.hasFallingEdge) {
    return false;
  }
  const bool turnSignalRecentlyChanged =
      nowMs - turnSignal.lastFallingEdgeMs < kTurnSignalWindowMs;
  const bool independentOfBrake =
      !brakeSignal.hasFallingEdge ||
      absDiffMs(turnSignal.lastFallingEdgeMs, brakeSignal.lastFallingEdgeMs) >
          kEdgeCoincidenceMs;
  return turnSignalRecentlyChanged && independentOfBrake;
}

// The setup function runs once when you press reset or power the board
void setup() {
  // Clear a pending watchdog reset so it does not fire again during setup.
  MCUSR = 0;
  wdt_disable();

  // Drive the outputs to the safe state before enabling them, so they never
  // glitch to the suppress level.
  digitalWrite(kLeftTurnSignalOutputPin, kAllowLevel);
  digitalWrite(kRightTurnSignalOutputPin, kAllowLevel);
  pinMode(kLeftTurnSignalOutputPin, OUTPUT);
  pinMode(kRightTurnSignalOutputPin, OUTPUT);

  const uint32_t nowMs = millis();
  brakeSignal.begin(nowMs);
  leftTurnSignal.begin(nowMs);
  rightTurnSignal.begin(nowMs);

  wdt_enable(WDTO_250MS);
}

// The loop function runs over and over again forever
void loop() {
  const uint32_t nowMs = millis();

  brakeSignal.update(nowMs);
  leftTurnSignal.update(nowMs);
  rightTurnSignal.update(nowMs);

  digitalWrite(kLeftTurnSignalOutputPin,
               isTurnSignalAllowed(leftTurnSignal, nowMs) ? kAllowLevel
                                                          : kSuppressLevel);
  digitalWrite(kRightTurnSignalOutputPin,
               isTurnSignalAllowed(rightTurnSignal, nowMs) ? kAllowLevel
                                                           : kSuppressLevel);

  wdt_reset();
}

// Behaviour per case:
// - Brake not active: turn signal output is always allowed.
// - Brake active, no turn signal: output is suppressed.
// - Turn signal blinking, then brake applied: allowed, because each blink's
//   falling edge re-opens the kTurnSignalWindowMs window.
// - Brake active, then turn signal starts: suppressed until the first blink's
//   falling edge, then allowed while blinking continues.
// - A turn line falling edge within kEdgeCoincidenceMs of a brake falling
//   edge (both lines dropping on brake release) is not counted as turn signal
//   activity, so re-applying the brake shortly after stays suppressed.
