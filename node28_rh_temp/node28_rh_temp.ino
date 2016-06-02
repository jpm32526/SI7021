 /*
 *   node_28.ino
 *   
 *   node28 rh + temp transmitter
 *   26Jan2016 jim maupin
 *
 * transmitter powered from external 12V supply, with LM78L33 regulator
 * so no need to monitor supply voltage.
 *
 *                       ATMega328 Pin Usage....                  
 *                            +---\/---+
 *                    reset  1|        |28  PC5 (ADC5 / D 19) SCL to Si7021
 *           RXD  (D 0) PD0  2|        |27  PC4 (ADC4 / D 18) SDA to Si7021
 *           TXD  (D 1) PD1  3|        |26  PC3 (ADC3 / D 17) 
 * RFM IRQ   INT0 (D 2) PD2  4|        |25  PC2 (ADC2 / D 16) 
 *           INT1 (D 3) PD3  5|        |24  PC1 (ADC1 / D 15) 
 *                (D 4) PD4  6|        |23  PC0 (ADC0 / D 14) 
 *                      Vcc  7|        |22  A_Gnd             Analog Gnd
 *                      Gnd  8|        |21  A_Ref             Analog Ref
 *              Crystal PB6  9|        |20  A_Vcc             Analog +V
 *              Crystal PB7 10|        |19  PB5 (D 13)  SCK   to radio
 *                (D 5) PD5 11|        |18  PB4 (D 12)  MISO  to radio       
 *                (D 6) PD6 12|        |17  PB3 (D 11)  MOSI  to radio 
 *                (D 7) PD7 13|        |16  PB2 (D 10)  SS    to radio
 *                (D 8) PB0 14|        |15  PB1 (D  9) 
 *                            +--------+
 */
//
//  -----[ Defines ]----------------------------------------------------
//
#define DEBUG 1
#define RF69_COMPAT 0 // define this to use the RF69 driver i.s.o. RF12
//
//  -----[ Include Libraries ]------------------------------------------
//
#include <JeeLib.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Si7021.h>
// 
//----------- radio setup parameters ----------------------------------
// 
#define MYNODE 28                 // node
#define FREQ RF12_433MHZ          // frequency
#define GROUP 212                 // network group
//
//  -----[ Variables ]--------------------------------------------------
// 
static uint8_t heaterOnOff = 0; // static variable for heater control
//
//  -----[ library init: ]----------------------------------------------
//
SI7021 si7021;
MilliTimer timer;
typedef struct {int airTemp , airHumidity;} PayloadTX;
PayloadTX outhouse;

//
//  -----[ Interrupt Service Routine ]----------------------------------
//
EMPTY_INTERRUPT(WDT_vect);      // just wakes us up to resume
//
//  -----[ Subroutines and Functions ]----------------------------------
//
static void watchdogInterrupts (char mode) {
  MCUSR &= ~(1 << WDRF);        // only generate interrupts, no reset
  cli();
  WDTCSR |= (1 << WDCE) | (1 << WDE); // start timed sequence
  WDTCSR = mode >= 0 ? bit(WDIE) | mode : 0;
  sei();
}
//---------------------------------------------------------------------
static void lowPower (byte mode) {
  // prepare to go into power down mode
  set_sleep_mode(mode);
  // disable the ADC
  byte prrSave = PRR, adcsraSave = ADCSRA;
  ADCSRA &= ~ bit(ADEN);
  PRR |=  bit(PRADC);
  // zzzzz...
  sleep_mode();
  // re-enable the ADC
  PRR = prrSave;
  ADCSRA = adcsraSave;
}
//---------------------------------------------------------------------
static byte loseSomeTime (word msecs) {
  // only slow down for periods longer than the watchdog granularity
  if (msecs >= 16) {
    // watchdog needs to be running to regularly wake us from sleep mode
    watchdogInterrupts(0); // 16ms
    for (word ticks = msecs / 16; ticks > 0; --ticks) {
      lowPower(SLEEP_MODE_PWR_DOWN); // now completely power down
      // adjust the milli ticks, since we will have missed several
      extern volatile unsigned long timer0_millis;
      timer0_millis += 16;
    }
    watchdogInterrupts(-1); // off
    return 1;
  }
  return 0;
}  /* End of power-saving code.  */
//
//  -----[ System Initilization ]---------------------------------------
// 
void setup() {
  si7021.begin();            // Runs : Wire.begin() + reset()
  rf12_initialize(MYNODE, RF12_433MHZ, GROUP); // 433 Mhz, net group 212, node 22
  si7021.setHumidityRes(12); // Humidity = 12-bit / Temperature = 14-bit
  si7021.setHeater(heaterOnOff); // Turn heater on or off

#if DEBUG
  Serial.begin(115200);
  Serial.print("\n[outhouse transmitter]");
  Serial.print("\n[node]");
  Serial.print(MYNODE);
  Serial.print(", [freq]");
  Serial.print(FREQ);
  Serial.print(", [group]");
  Serial.print(GROUP);
  Serial.print("\nHeater Status = ");
  Serial.print(si7021.getHeater() ? "ON\n" : "OFF\n");
#endif
}   /*  End setup   */

//  -----[ Main Program Loop ]------------------------------------------
// 
void loop() {
  // spend most of the waiting time in a low-power sleep mode
  // note: the node's sense of time is no longer 100% accurate after sleeping
  rf12_sleep(RF12_SLEEP);          // turn the radio off
  loseSomeTime(timer.remaining()); // go into a (controlled) comatose state

  while (!timer.poll(60000))       // one minute give or take...
    lowPower(SLEEP_MODE_IDLE);     // still not running at full power
  rf12_sleep(RF12_SLEEP);          // turn the radio off

  {
    outhouse.airTemp     = int(((si7021.readTemp() * 1.8) + 32) * 100);
    outhouse.airHumidity = int(si7021.readHumidity() * 100);
#if DEBUG
    Serial.print("\nHumidity : ");
    Serial.print(outhouse.airHumidity * .01); // print humidity to serial monitor
    Serial.print(" %\t");
    Serial.print("Temp : ");
    Serial.print(outhouse.airTemp * .01);     // print to temp to serial monitor
    Serial.print(" F");
#endif

    rf12_sleep(RF12_WAKEUP);         // turn radio back on at the last moment
    MilliTimer wait; 
    while (!wait.poll(5))
    {
      rf12_recvDone();
      lowPower(SLEEP_MODE_IDLE);

      while (!rf12_canSend())
        rf12_recvDone();

      rf12_sendNow(0, &outhouse, sizeof outhouse);
      rf12_sendWait(0);
    }
  }
}   /*  End of main loop  */

