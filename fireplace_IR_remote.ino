/*
  Fireplace Remote Control and Sleep Timer

  IR Remote control of a gas fireplace

  Feature Brainstorming:

  - "Learning Mode" - enabled when powered on.  Can learn new remote codes.
    - Probably should store/retrieve the current IR codes in EEPROM
  - LED display and feedback
  - "flicker" when IR signal is detected
	- display LEDs to indicate "Off Timer" setting
	- slow blink when within 2 minutes of shutdown

  - "Thermostat" mode.  Hook up a temperature sensor and turn on/off based on temperature
  - Manual operation button in case you lose the IR.  Allows you to simulate the IR button presses (I assume I only have one IR button?)
    I assume I could just call the same subroutines as I do for the IR presses and just "fake" the code?
  - Logic to avoid cycling on/off too quickly?  Certainly I could impart a delay before starting/stopping the fireplace, but I'd need
    to make sure I have some sort of visual feedback that the command was accepted, otherwise folks will probably just bang on the button.
  - Failsafe - check that timer is never over 60?  Turn off after 4 hours max?


  - DONE - Sleep Timer - turn off the fireplace after 15, 30, 45, 60 minutes
  - DONE - Manual ON switch (for when microcontroller is powered down)
  - DONE -   - "Arm" position, or "Active" when microntroller is enabled
	- "OFF" - system completely disconnected.  Fireplace cannot be turned on.
  - DONE   - Should keep track of the fireplace state so you don't try to turn on an already on fireplace.  Not sure if this is
            really required or not, but I think the valve got a bit pissy when I turned it on twice in a row???  Need to test.

  - Filtering capacitors (0.1 uF ?)

  ISP Programming Info

  Vcc  = Physical Pin  #1
  RST  = Physical Pin  #4
  MOSI = Physical Pin  #7
  MISO = Physical Pin  #8
  SCK  = Physical Pin  #9
  GND  = Physical Pin #14
 */

#include <IRremote.h>                 // IR remote control library
#include <SoftSerial.h>               // Software Serial library to help with debugging (for ATtiny)
#include <SoftwareSerial.h>           // Standard Software Serial library (for ATmega)

// #include <Time.h>
// #include <TimeAlarms.h>

const int debounceDelay = 10;          // milliseconds to wait until stable reading of pushButton

const int CONSOLE = 1;                 // "made up" numbers used to indicator the "source" of the event
const int IR = 2;                      // This is the IR "made up" source number
const int MINUTES_PER_INDICATOR = 15;  // Each indicator light = 15 minutes
const int BUTTON_PRESS_DELAY = 3000;   // number of milliseconds after which I'll assume the user is done pressing buttons
unsigned long lastUserInput = 0;       // This tracks the last time user input was received.  Used in conjunction with BUTTON_PRESS_DELAY to determine when to commit a command.
const unsigned int MS_PER_MIN = 60000; // 60000 milliseconds per minute. This allows me to adjust the timing for either testing purposes or inaccurate crystals


#if defined(__AVR_ATtiny84__)
const int rxpin =  10;                 // PB0, Physical Pin #2 - Software Serial receive  pin - not used
const int txpin =   9;                 // PB1, Physical Pin #3 - Software Serial transmit pin

// LED Indicators for 0 min, 15 min, 30 min, 45 min, and 60 minutes
const int led00         = 99;          // Bogus or unused
const int led15         = 0; 	       // PA0, Physical Pin #13  Should be a PWM capable pin for warningLight
const int led30         = 1; 	       // PA1, Physical Pin #12
const int led45         = 2; 	       // PA2, Physical Pin #11
const int led60         = 3; 	       // PA3, Physical Pin #10

const int fireplacePin  = 4;           // PA4, Physical Pin #9 - pin to activate optoisolator to turn on fireplace
const int irReceivePin  = 5;           // PA5, Physical Pin #8 - pin connected to IR detector output
const int pushButton    = 6;           // PA6, Physical Pin #7 - hooked to MOM-PB. Enable PullUP resistor!  
                                       // the value on the pin goes LOW when the button is pressed
#else

// ATmega328 Section Specific

const int rxpin =   0;
const int txpin =   1;  // Note here that I'm using the software serial onto my hardware serial port.  This seems to work!

// LED Indicators for 15 min, 30 min, 45 min, and 60 minutes
const int led00         = 99;   // Bogus or unused
const int led15         =  6; 	// Should be a PWM capable pin for warningLight
const int led30         =  9; 	
const int led45         = 10; 	
const int led60         = 11; 	

const int fireplacePin  = 4;    // 
const int irReceivePin  = 5;    // 
const int pushButton    = 2;    // This is Interrupt 0 on the ATmega (Arduino Pin 2)

#endif

const int numberOfKeys  = 2;             //  How many different key codes I can store.  Probably only need two. (0 through 1)
long irKeyCodes[numberOfKeys];           // holds the codes for each key

const int numIndicators = 5;             // Four visible, one is "blank" or "all off"
int indicatorArrayIndex = 0;             // Keeps track of which LED is lit (index into indicatorArray)
int indicatorArray [numIndicators] = {led00, led15, led30, led45, led60};

/*
const char GARBAGE1 [] = "What the heck is this and why should I even care?";
const char GARBAGE2 [] = "What the heck is this and why should I even care?";
const char GARBAGE3 [] = "What the heck is this and why should I even care?";
const char GARBAGE4 [] = "What the heck is this and why should I even care?";
const char GARBAGE5 [] = "What the heck is this and why should I even care?";
const char GARBAGE6 [] = "What the heck is this and why should I even care?";
const char GARBAGE7 [] = "What the heck is this and why should I even care?";
const char GARBAGE8 [] = "What the heck is this and why should I even care?";
*/


boolean fireplaceOn = false;                  // keep track of fireplace state
int fireplaceMinutes = 0;                     // Number of minutes left to run fireplace. Start with 0 so you don't FLAME ON!  :-)  Unsigned, so never negative
const int SHUTDOWN_WARNING_MINUTES = 13;      // Flash the LED when you've got this many minutes left
boolean   SHUTDOWN_WARNING_PERIOD = false;    // Am I in the "WARNING" period for imminent shutdown?
const int CYLON_DELAY_PERIOD = 20;            // Adjust how fast the WARNING light cycles - 20ms seems about right

// Safety section
const int8_t MAX_FIREPLACE_TIMER_VALUE = 60;  // You should never find a timer value larger than this
const int8_t MAX_FILEPLACE_TIME = 120;        // Fireplace will never be on longer than this amount of time

// These are used in the interrupt service routine, so they are volatile
volatile unsigned long buttonPressDuration = 0;  // Also used as a flag that a button press now requires processing

unsigned long elapsedWholeMinutes = 0;        // Number of whole minutes since the program started.  unsigned long gives me 4 bytes (2 ^ 32 = ~8000 years without a rollover)
unsigned long tmpWholeMinutes;                // tmp location to stash calculation so I don't have to test condition and calculate twice

// Initialize IR library and storage location
IRrecv irrecv (irReceivePin);         // create the IR library (I believe this is actually a function prototype)
decode_results results;               // returned IR data goes here
SoftwareSerial debug (rxpin, txpin);  // create a new software serial port

void setup()
{
  debug.begin(9600);
  
 /*
  debug.println (GARBAGE1);
  debug.println (GARBAGE2);
  debug.println (GARBAGE3);  
  debug.println (GARBAGE4);  
  */
 
  pinMode (led15, OUTPUT);
  pinMode (led30, OUTPUT);
  pinMode (led45, OUTPUT);
  pinMode (led60, OUTPUT);

  pinMode (fireplacePin, OUTPUT);
  pinMode (irReceivePin, INPUT);
  pinMode (pushButton, INPUT);         // This is the manual operation button
  digitalWrite (pushButton, HIGH);     // turn on internal pull-up (20K - 50K) on pushButton (avoids a resistor, but reverses logic)
  
  irrecv.enableIRIn ();                // Start the IR receiver

  // Debugging Stuff - might want to try for coditional compilation of this down the road
  delay (1000);  // Let things "settle" for a bit - otherwise it's garbage time
  debug.println ();  // seems like it wants to munge the first message
  debug.println ();  // send a second CR to start on a clear line
  
  #if defined(__AVR_ATtiny84__)
    debug.println ("Started fireplace Remote Control Sketch on ATtiny84");
  #else
    debug.println ("Started fireplace Remote Control Sketch on ATmega328");
  #endif

  // debug.println ("Startup Delay ...");
  // delay (2000);
  
  debug.println ("Armed and ready."); 
   
  // Set up interrupt to handle buttonPress
  attachInterrupt (0, isrButtonPress, CHANGE);   
  buttonPressDuration = 0;    // For some reason the above interrupt "fires" when it is first attached.  At least I believe that is the case.  Setting this var to "0" fixes that issue.
  
  // learnKeycodes();                  // learn remote control key  codes
}

void loop()
{
  long key;
   
  // The logic should be the same regardless if you pressed the physical button or the remote control button
  // So I will put the control logic into a subroutine which gets called from either action
  
  if (buttonPressDuration > 0)   // Will fire if the time is greater than 0
  {
    buttonPress (CONSOLE);
    buttonPressDuration = 0;  // reset this back to zero since I use it as a flag for processing    
  }  

  if (irrecv.decode(&results))  // checking if there is any IR data in the hopper.  The what?  "THE HOPPA!"  "SHAH-DUP!!!"
  {
    if (results.value != -1)    // I'm guess that -1 indicates some problem with the results - will have to check to be sure.
    {
      key = results.value;      
      showReceivedData();
  
      if((key == 0x801192A8) || (key == 0x801112A8))
      {
        buttonPress (IR);
      }
    } 
    irrecv.resume();
  }
 
  if (millis () - lastUserInput > BUTTON_PRESS_DELAY)     // Wait until the user stops pressing buttons
  {
    if ( ! (fireplaceOn) && (fireplaceMinutes > 0) )      // if the fireplace is off AND the fireplaceMinutes > 0, then turn on fireplace
    {
      digitalWrite (fireplacePin, HIGH);
      debug.println ("Sending START signal to fireplace");
      fireplaceOn = true;
    }
    else if ( (fireplaceOn) && (fireplaceMinutes <= 0) )  // if the fireplace is on AND the fireplaceMinutes <=0, then turn off the fireplace
    {
      digitalWrite (fireplacePin, LOW);      
      debug.println ("Sending STOP signal to fireplace");
      fireplaceOn = false;
    }
  }
 
  
  // Do the glowing LED thang when the fireplace is about to go out. 
  if (SHUTDOWN_WARNING_PERIOD) warningLight ();
       
  // I want to call the "countDownTimer" routine once a minute
  if ( (tmpWholeMinutes = millis () / MS_PER_MIN) > elapsedWholeMinutes)   // this means we've advanced at least a minute 
  {
    // debug.print ("advancing minutes.  elapsedWholeMinutes =      ");
    // debug.println (elapsedWholeMinutes);    
    countDownTimer ();
    elapsedWholeMinutes = tmpWholeMinutes;  // update with the lastest value (which I stored in tmpWholeMinutes
  }
  
}

/*

 * This takes care of handling the time based events for the fireplace
 * 1. decrementing the fireplaceMinutes
 * 2. adjusting the indicator lights
 * 3. turning the fireplace off when needed

 */
void countDownTimer ()
{
  
  if (fireplaceOn)  // All of the logic below is only of interest if the fireplaceOn == true
  {
    if (fireplaceMinutes == 0)
    {
      debug.println ("Turning Fireplace OFF");
      digitalWrite (indicatorArray[indicatorArrayIndex], LOW);  // turn off last indicator
      indicatorArrayIndex = 0;
      fireplaceOn = false;
      SHUTDOWN_WARNING_PERIOD = false;   // disable the CYLON glowing LED
      // TURN OFF THE FIREPLACE HERE
      digitalWrite (fireplacePin, LOW);   // turned it off
    }
    else if (fireplaceMinutes <= SHUTDOWN_WARNING_MINUTES)  // Dang, it's about to go out!
    {
      fireplaceMinutes -= 1;
      SHUTDOWN_WARNING_PERIOD = true;   // enable the CYLON glowing LED
      debug.print ("WARNING: Fireplace is about to go off!  Time remaining: ");
      debug.println (fireplaceMinutes);
    }
    else // if (fireplaceOn == true)   // If the fireplace isn't yet on, don't report that it's running.  [Needed this logic only during testing when 1 minute = 5 seconds]
    {
      fireplaceMinutes -= 1;
      SHUTDOWN_WARNING_PERIOD = false;   // disable the CYLON glowing LED if enabled      
      debug.print ("Fireplace is running.  Time remaining: ");
      debug.println (fireplaceMinutes);    
      if (fireplaceMinutes + MINUTES_PER_INDICATOR < (indicatorArrayIndex * MINUTES_PER_INDICATOR))
      {
        digitalWrite (indicatorArray[indicatorArrayIndex--], LOW);  // turn off current indicator light
        digitalWrite (indicatorArray[indicatorArrayIndex], HIGH);   // turn on next one
        debug.print ("Cycling to new indicator light number ");   
        debug.println (indicatorArrayIndex);   
      }
    }
  }
}


/*
 * button pressed
 *
 * Pulling this logic out into its own function since the behavior is the same for a physical button press as it 
 * is for an IR button press
 */
 
void buttonPress (int source)
{
  // source is a text string indicating "console" or "IR" - not sure if I need that info or not
  lastUserInput = millis ();  // record when the last button was pressed so I can decide when the user is "done"
  
  SHUTDOWN_WARNING_PERIOD = false;   // cancel the CYLON glowing LED (only really needed if in SHUTDOWN_WARNING_PERIOD)

  /*  This is pretty much debugging, so commenting it out for now
  if (source == CONSOLE)
  {
    debug.print ("CONSOLE button was depressed for ");
    debug.print (buttonPressDuration);
    debug.println (" milliseconds");
  }
  else  // I will assume this was an "IR" source, although I could test for it if needed.
  {
    debug.println ("IR button was pressed");
  }
  */
    
  // turn off current indicator and turn on the next one
  digitalWrite (indicatorArray[indicatorArrayIndex++], LOW);
  if (indicatorArrayIndex == numIndicators)    // zero based index vs. physical numbers, so this works without a "-1" correction
  {
    indicatorArrayIndex = 0;  // wraps value to stay below max index
  }
  digitalWrite (indicatorArray[indicatorArrayIndex], HIGH);
  
  // Adjust fireplaceMinutes to newly selected value
  fireplaceMinutes = indicatorArrayIndex * MINUTES_PER_INDICATOR;  // Each indicator segment is MINUTES_PER_INDICATOR long
  debug.print ("fireplaceMinutes is now set to ");
  debug.println (fireplaceMinutes);
}

/*
 * get remote control codes
 */
void learnKeycodes()
{
  while(irrecv.decode(&results))   // empty the buffer
    irrecv.resume();
 
  debug.println("Ready to learn remote codes");
  long prevValue = -1;
  int i=0;
  while( i < numberOfKeys)
  {
    debug.print("press remote key ");
    debug.print(i);
    while(true)
    {
      if( irrecv.decode(&results) )
      {
          if(results.value != -1 && results.value != prevValue)
          {
            showReceivedData();
            irKeyCodes[i] = results.value;
            i = i + 1;
            prevValue = results.value;
            irrecv.resume(); // Receive the next value
            break;
          }
        irrecv.resume(); // Receive the next value
      }
    }
  }
  debug.println("Learning complete");
}

/*
 * converts a remote protocol code to a logical key code 
 * (or -1 if no digit received)
 */
int convertCodeToKey(long code)
{
  for( int i=0; i < numberOfKeys; i++)
  {
    if( code == irKeyCodes[i])
    {
      return i; // found the key so return it
    }
  }
  return -1;
}

/*
 * display the protocol type and value
 */
void showReceivedData()
{
  if (results.decode_type == UNKNOWN)
  {
    debug.println("-Could not decode message");
  }
  else
  {
    if (results.decode_type == NEC) {
      debug.print("- decoded NEC: ");
    }
    else if (results.decode_type == SONY) {
      debug.print("- decoded SONY: ");
    }
    else if (results.decode_type == RC5) {
      debug.print("- decoded RC5: ");
    }
    else if (results.decode_type == RC6) {
      debug.print("- decoded RC6: ");
    }
    debug.print("hex value = ");
    debug.println( results.value, HEX);
  }
}

// return the time in milliseconds that the pushButton has been in pressed (LOW in my case)
// 
long switchTime(int pin)
{
  static unsigned long startTime = 0;       // the time the switch state change was first detected
  static boolean state;                     // the current state of the switch

  if (digitalRead (pin) != state)    // check to see if the switch has changed state
  {
    state = ! state;                        // yes, invert the state
    startTime = millis();                   // store the time
  }

  if( state == LOW)
    return millis() - startTime;            // switch pushed, return time in milliseconds
  else
    return 0;                               // return 0 if the switch is not pushed (i.e. in the HIGH state);
}


// Interrupt Service Routine for buttonPress
void isrButtonPress ()
{ 
  
  static unsigned long lastInterruptTime = 0;
  volatile static unsigned long buttonPressBegin;  // This is only used locally within the ISR (thus volatile), but needs to be preserved call over call (i.e. static)
  // Debounce the button press [without the use of delay (since I can't use that here)]
  // Basically I just ignore any interrupt that occurs too quickly (i.e. less than debounceDelay)
  //  static unsigned long lastInterruptTime = millis (); // This tracks how long ago this ISR was called.  I used to have this set to 0, but the program was detecting a button press upon powerup, so trying this.
                                                      // The above modification seems to have done the trick
  if (millis () - lastInterruptTime > debounceDelay)  // If this is true, then go ahead and "do stuff". Otherwise, just exit.
  {
    int buttonState = digitalRead (pushButton);       // get the current state of the button
    
    if (buttonState == HIGH)                          // This is a transition from LOW -> HIGH, which means the button got released
      buttonPressDuration = millis () - buttonPressBegin;
    else                                              // This must be a HIGH -> LOW transition, which means the button got pressed
      buttonPressBegin = millis ();

    lastInterruptTime = millis ();                    // resets the lastInterruptTime
  }
}


/*
 * This is used to create a glowing indicator instead of a harsh flashing.
 * I'm going to use this for the WARNING period. 
 */

void warningLight () 
{
  static unsigned long lastAdjustmentTime = 0;  // to avoid a "delay" in the code, I'm going to keep track of the last time I changed the LED
  static uint8_t hbval   = 10;                  // Starts at the minimum value since the LED will already be illuminated when I start.  BE CAREFUL - you don't want to bump the top or the bottom value or you'll spill over to a negative value!
  static int8_t  hbdelta = 2;                   // Tried 4, but it seemed a bit fast.  Also be aware that this "step" value affects the "speed" of the cycle (see CYLON_DELAY_PERIOD)

  if (millis () - lastAdjustmentTime > CYLON_DELAY_PERIOD)  // CYLON DELAY adjust how quickly I cycle
  {
    if (hbval > 192) hbdelta = -hbdelta;        // started with 192
    if (hbval <  10) hbdelta = -hbdelta;        // started with  32
    hbval += hbdelta;
    analogWrite(led15, hbval);

    #ifdef DEBUGALL
   
      debug.print ("CYLON adjusted to ");
      debug.println (hbval);
      
    #endif

    lastAdjustmentTime = millis ();    

  }
}
