#include <Arduino.h>
#include <stdint.h>

// Define pin number
int ledPin = 13;
// Define timer counter
volatile uint32_t timer1Counter = 0;

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
  if (timer1CounterVal - lastToggle >= 100) {      // 100 * 10ms = 1000ms
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Toggle LED on pin 13
    lastToggle = timer1CounterVal;
  }
}