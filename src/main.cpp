#include <Arduino.h>
#include <stdint.h>

// Define pin number
int ledPin = 13;

int brakeSignalPin = 2;
int leftTurnSignalInputPin = 3;
int rightTurnSignalInputPin = 4;
int leftTurnSignalOutputPin = 5;
int rightTurnSignalOutputPin = 6;

// Define timer counter
volatile uint32_t timer1Counter = 0;

decltype(timer1Counter) lastLeftTurnSignalChangeHL = 0;
decltype(timer1Counter) lastRightTurnSignalChangeHL = 0;
decltype(timer1Counter) lastBrakeSignalChangeHL = 0;
uint8_t lastLeftTurnSignalState = LOW;
uint8_t lastRightTurnSignalState = LOW;
uint8_t lastBrakeSignalState = LOW;

void updateLastStates();

class InterruptLock {
public:
  InterruptLock() { cli(); }  // Disable global interrupts
  ~InterruptLock() { sei(); } // Enable global interrupts
};

decltype(timer1Counter) getTimer1Counter() {
  auto lock = InterruptLock{};
  return timer1Counter;
}

// The setup function runs once when you press reset or power the board
void setup() {
  // Initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);

  // Set up the timer interrupt
  auto lock = InterruptLock{};
  // Set up Timer1
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Same for TCCR1B

  // Set compare match register to desired timer count:
  // 16 MHz Clock / 64 prescaler = 250,000 counts per second
  // 250,000 / 100 = 2,500 counts for 10 ms
  OCR1A = 2499; // = 2500 - 1 (as timer count starts from 0)

  // Turn on CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS11 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

// Timer1 interrupt service routine called every 10ms
ISR(TIMER1_COMPA_vect) { timer1Counter++; }

// The loop function runs over and over again forever
void loop() {
  // Main loop code
  // Example: Toggle an LED every second
  static decltype(timer1Counter) lastToggle = 0;
  auto timer1CounterVal = getTimer1Counter();
  if (timer1CounterVal - lastToggle >= 100) { // 100 * 10ms = 1000ms
    // digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Toggle LED on pin 13
    lastToggle = timer1CounterVal;
  }

  // Turn signal logic
  // const auto leftTurnSignalState = digitalRead(leftTurnSignalInputPin);
  // const auto rightTurnSignalState = digitalRead(rightTurnSignalInputPin);
  const auto brakeSignalState = digitalRead(brakeSignalPin);

  auto allowLeftTurnSignal =
      brakeSignalState == LOW || // Brake signal is not active
      ((timer1CounterVal - lastLeftTurnSignalChangeHL) <
           80 && // Turn signal
                 // change was
                 // less than 400ms
       abs(lastLeftTurnSignalChangeHL - lastBrakeSignalChangeHL) >
           5); // Brake
               // signal
               // change
               // diff more
               // than 50ms

  auto allowRightTurnSignal =
      brakeSignalState == LOW ||
      ((timer1CounterVal - lastRightTurnSignalChangeHL) < 80 &&
       abs(lastRightTurnSignalChangeHL - lastBrakeSignalChangeHL) > 5);

  // allowLeftTurnSignal = true;
  // allowRightTurnSignal = true;
  digitalWrite(leftTurnSignalOutputPin, allowLeftTurnSignal ? LOW : HIGH);
  digitalWrite(rightTurnSignalOutputPin, allowRightTurnSignal ? LOW : HIGH);

  updateLastStates();
}

void updateLastStates() {
  const auto timer1CounterVal = getTimer1Counter();

  const auto leftTurnSignalState = digitalRead(leftTurnSignalInputPin);
  const auto rightTurnSignalState = digitalRead(rightTurnSignalInputPin);
  const auto brakeSignalState = digitalRead(brakeSignalPin);

  if (leftTurnSignalState == LOW && lastLeftTurnSignalState == HIGH) {
    lastLeftTurnSignalChangeHL = timer1CounterVal;
  }
  if (rightTurnSignalState == LOW && lastRightTurnSignalState == HIGH) {
    lastRightTurnSignalChangeHL = timer1CounterVal;
    // digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Toggle LED on pin 13
  }
  if (brakeSignalState == LOW && lastBrakeSignalState == HIGH) {
    lastBrakeSignalChangeHL = timer1CounterVal;
  }

  lastLeftTurnSignalState = leftTurnSignalState;
  lastRightTurnSignalState = rightTurnSignalState;
  lastBrakeSignalState = brakeSignalState;
}

// Possible cases:
// - Turn signal starts withouth brake signal
// - Turn signal starts withouth brake signal and brake signal starts
// - Turn signal starts with brake signal
