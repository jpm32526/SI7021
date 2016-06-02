/*
   node8 rh + temp transmitter
   26Jan2016 jim maupin
   From JeeLab and Si7021 demo programs

   BASIC DEMO
   ----------
   Print humidity and temperature to the serial monitor and xmit via RFM12B

   Uses M328 powered by LiPo battery + MCP1702-33 regulator and RFM69CW,
   measure battery with a voltage divider 
   4M7-10M + 100nF cap to maintain voltage for adc. 
   
*/

#define DEBUG 1
#define RF69_COMPAT 1 // define this to use the RF69 driver i.s.o. RF12
//---------------------------------------------------------------------
//-------------- Library declarations ---------------------------------
//---------------------------------------------------------------------
#include <JeeLib.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Si7021.h>
//---------------------------------------------------------------------
//----------- radio setup parameters ----------------------------------
//---------------------------------------------------------------------
#define MYNODE 8                  // node
#define FREQ RF12_433MHZ          // frequency
#define GROUP 212                 // network group
//---------------------------------------------------------------------
//----------- Initialize Objects --------------------------------------
//---------------------------------------------------------------------
SI7021 si7021;
MilliTimer timer;

typedef struct {
  int battery, airTemp , airHumidity;
} PayloadTX;
PayloadTX outhouse;

//---------------------------------------------------------------------
//----------- Program variables ---------------------------------------
//---------------------------------------------------------------------
static uint8_t heaterOnOff = 0; // static variable for heater control
//---------------------------------------------------------------------
//----------- Power Management ----------------------------------------
//---------------------------------------------------------------------
EMPTY_INTERRUPT(WDT_vect);      // just wakes us up to resume

//---------------------------------------------------------------------
static void watchdogInterrupts (char mode) {
  MCUSR &= ~(1 << WDRF);        // only generate interrupts, no reset
  cli();
  WDTCSR |= (1 << WDCE) | (1 << WDE); // start timed sequence
  WDTCSR = mode >= 0 ? bit(WDIE) | mode : 0;
  sei();
}
//---------------------------------------------------------------------
static void lowPower (byte mode) {
  set_sleep_mode(mode);            // prepare to go into power down mode
  byte prrSave = PRR, adcsraSave = ADCSRA;            // disable the ADC
  ADCSRA &= ~ bit(ADEN);
  PRR |=  bit(PRADC);

  sleep_mode();                    // zzzzz...

  PRR = prrSave;                   // interrupted, re-enable the ADC
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

//---------------------------------------------------------------------
//----------- Initialize Program --------------------------------------
//---------------------------------------------------------------------
void setup() {
  si7021.begin(); // Runs : Wire.begin() + reset()
  rf12_initialize(MYNODE, RF12_433MHZ, GROUP); // node 22, 433 MHz, group 212
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
//---------------------------------------------------------------------
//----------- Main Program Loop ---------------------------------------
//---------------------------------------------------------------------
void loop() {
  // spend most of the waiting time in a low-power sleep mode
  // note: the node's sense of time is no longer 100% accurate after sleeping
  rf12_sleep(RF12_SLEEP);          // turn the radio off
  loseSomeTime(timer.remaining()); // go into a (controlled) comatose state

  while (!timer.poll(60000))       // one minute give or take...
    lowPower(SLEEP_MODE_IDLE);     // still not running at full power
  rf12_sleep(RF12_SLEEP);          // turn the radio off

    outhouse.battery     = int(analogRead(0) * 4.6544);  // using external divider
    outhouse.airTemp     = int(((si7021.readTemp() * 1.8) + 32) * 100);
    outhouse.airHumidity = int(si7021.readHumidity() * 100);
#if DEBUG
    Serial.print("\nHumidity : ");
    Serial.print(outhouse.airHumidity * .01); // copy struct data, print to serial
    Serial.print(" %\t");
    Serial.print("Temp : ");
    Serial.print(outhouse.airTemp * .01);
    Serial.print(" F\t");
    Serial.print(" Battery ");
    Serial.print(outhouse.battery * 0.001);
#endif

    rf12_sleep(RF12_WAKEUP);         // turn radio back on at the last moment
    MilliTimer wait;                 // radio needs some time to power up, why?
    while (!wait.poll(5))
    {
      rf12_recvDone();
      lowPower(SLEEP_MODE_IDLE);

      while (!rf12_canSend())
        rf12_recvDone();

      rf12_sendNow(0, &outhouse, sizeof outhouse); // sync mode!
      //      rf12_sendWait(2);      // if not disabled, transmits twice. WTF????
    }
}   /*  End of main loop  */

