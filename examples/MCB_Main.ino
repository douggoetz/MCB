/*
 *  MCB_Main.ino
 *  Main file for the Motor Control Board
 *  Author: Alex St. Clair
 *  Created: December, 2017
 *  
 *  This is the main file that should be used to program
 *  the motor control board (MCB) for FLOATS and RACHuTS on
 *  the Strateole 2 flights.
 */
#include <HardwareMCB.h>
#include <MCB.h>
#include <ConfigManagerMCB.h>
#include <TimerOne.h>

#define TIMER_COUNTER_MAX   10 // loop_rate = 10 / TIMER_COUNTER_MAX Hz

// Main MCB object ------------------------------------------------------------
MCB mcb;

// Timer variables ------------------------------------------------------------
volatile uint8_t timer_counter;
volatile bool controlLoopTimerFlag = true;

// Timer ISR for control loop -------------------------------------------------
void ControlLoopTimer(void) {
  if (++timer_counter == TIMER_COUNTER_MAX) {
    timer_counter = 0;
    controlLoopTimerFlag = true;
  }

  // LED heartbeat
  if (timer_counter == 0) digitalWrite(PULSE_LED_PIN, LOW);
  if (timer_counter == 1) digitalWrite(PULSE_LED_PIN, HIGH);
  if (timer_counter == 2) digitalWrite(PULSE_LED_PIN, LOW);
  if (timer_counter == 3) digitalWrite(PULSE_LED_PIN, HIGH);
}

// Wait for control timer interrupt routine -----------------------------------
void WaitForControlTimer(void) {
  bool status = false;
  while (status == false) {
    delay(1);
    noInterrupts();
    if (controlLoopTimerFlag) {
      controlLoopTimerFlag = false;
      status = true;
    }
    interrupts();
  }
}

// Standard Arduino setup -----------------------------------------------------
void setup() {
  // Pulse LED setup
  pinMode(PULSE_LED_PIN, OUTPUT);
  digitalWrite(PULSE_LED_PIN, LOW);

  // Timer interrupt setup for main loop timing
  Timer1.initialize(100000); // 0.1 s
  Timer1.attachInterrupt(ControlLoopTimer);

  // MCB class initialization
  mcb.Startup();
  delay(500);
}

// Main loop ------------------------------------------------------------------
void loop() {
  // MCB class loop
  mcb.Loop();

  
  // wait for loop timing interrupt
  WaitForControlTimer();
}
