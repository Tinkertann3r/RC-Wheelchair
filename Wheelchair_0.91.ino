
// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//
//  Pulse width modulation info https://www.arduino.cc/en/Reference/AnalogWrite
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>


// Assign your channel in pins
#define FORWARD_REVERSE_PIN 7
#define RIGHT_LEFT_PIN 8
#define AUX_IN_PIN 12

// Assign your channel out pins
#define FORWARD_REVERSE_OUT_PIN 3
#define FORWARD_REVERSE_OUT_INV_PIN 9
#define RIGHT_LEFT_OUT_PIN 10
#define RIGHT_LEFT_OUT_INV_PIN 11
#define AUX_OUT_PIN 6


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define FORWARD_REVERSE_FLAG 1
#define RIGHT_LEFT_FLAG 2
#define AUX_FLAG 4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unForwardReverseInShared;
volatile uint16_t unRightLeftInShared;
volatile uint16_t unAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulForwardReverseStart;
uint32_t ulRightLeftStart;
uint32_t ulAuxStart;
uint32_t timer = 0;

  // define variables for later
  int ForwardReverse;
  int ForwardReverseInv;
  float ForwardReversePWM;
  int RightLeft;
  int RightLeftInv;
  float RightLeftPWM;

void setup()
{
  Serial.begin(9600);
  
  Serial.println("rOcK oN");

  // set PWM pins as outputs

  pinMode(FORWARD_REVERSE_OUT_PIN, OUTPUT);
  pinMode(FORWARD_REVERSE_OUT_INV_PIN, OUTPUT);
  pinMode(RIGHT_LEFT_OUT_PIN, OUTPUT);
  pinMode(RIGHT_LEFT_OUT_INV_PIN, OUTPUT);
  pinMode(AUX_OUT_PIN, OUTPUT);

 // set pins to 2.5 volts, joystick centered
  analogWrite(FORWARD_REVERSE_OUT_PIN, 128);
  analogWrite(FORWARD_REVERSE_OUT_INV_PIN, 128);
  analogWrite(RIGHT_LEFT_OUT_PIN, 128);
  analogWrite(RIGHT_LEFT_OUT_INV_PIN, 128);
  // analogWrite(AUX_OUT_PIN, 128)
  

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(FORWARD_REVERSE_PIN, calcForwardReverse,CHANGE); 
  PCintPort::attachInterrupt(RIGHT_LEFT_PIN, calcRightLeft,CHANGE); 
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE); 

   delay(50);  // add delay to allow trasmitter to power up and prevent bad readings
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop.
  static uint16_t unForwardReverseIn = 1480;  //initialize to neutral position to prevent unintended motion
  static uint16_t unRightLeftIn = 1480;
  static uint16_t unAuxIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  
  // set all output to neutral if no signal - FAIL SAFE FOR LOST RECEPTION

  if(millis()-timer > 1000)
  {
   Serial.println("No Signal");

   unRightLeftIn = 1480;
   unForwardReverseIn = 1480;
   
  }

  // check shared update flags to see if any channels have a new signal
  
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
   
    timer = millis();
    
    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    
    if(bUpdateFlags & FORWARD_REVERSE_FLAG)
    {
      unForwardReverseIn = unForwardReverseInShared;
    }
    
    if(bUpdateFlags & RIGHT_LEFT_FLAG)
    {
      unRightLeftIn = unRightLeftInShared;
    }
    
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the 
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unAuxIn, unForwardReverseIn and unRightLeftIn, the shared
  // variables unAuxInShared, unForwardReverseInShared, unRightLeftInShared are always owned by 
  // the interrupt routines and should not be used in loop
  
  // we are checking to see if the channel value has changed, this is indicated  
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.

  // Radio signal; 1056 = full reverse; 1480 = neutral, 1884 = full forward
  // Output PMW; 77 = Full Reverse, 128 = neutral, 179 = full forward

  if (unForwardReverseIn < 1500 && unForwardReverseIn > 1450)  // Create deadband
  {
    ForwardReverse = 128;
    ForwardReverseInv = 128;
    analogWrite(FORWARD_REVERSE_OUT_PIN, ForwardReverse);
    analogWrite(FORWARD_REVERSE_OUT_INV_PIN, ForwardReverseInv);
    
    Serial.println(ForwardReverse);
  }
  
  else
  {
    ForwardReversePWM = (unForwardReverseIn-1056)/8.11;
    ForwardReverse = 77 + (int) ForwardReversePWM;
    ForwardReverseInv = 179 - (int) ForwardReversePWM;
    analogWrite(FORWARD_REVERSE_OUT_PIN, ForwardReverse);
    analogWrite(FORWARD_REVERSE_OUT_INV_PIN, ForwardReverseInv);

    Serial.println(ForwardReverse);
    // Serial.println(unForwardReverseIn);
    // Serial.println(ForwardReverse);

    
    
   }
   
  if (unRightLeftIn < 1500 && unRightLeftIn > 1450)  // Create deadband
  {
    RightLeft = 128;
    RightLeftInv = 128;
    analogWrite(RIGHT_LEFT_OUT_PIN, RightLeft);
    analogWrite(RIGHT_LEFT_OUT_INV_PIN, RightLeftInv);
    
    Serial.println(RightLeft);
   // Serial.println(unRightLeftIn);
    

  }

  else
  {
    RightLeftPWM = (unRightLeftIn-1056)/8.11;
    RightLeft = 77 + (int) RightLeftPWM;
    RightLeftInv = 179 - (int) RightLeftPWM;
    analogWrite(RIGHT_LEFT_OUT_PIN, RightLeft);
    analogWrite(RIGHT_LEFT_OUT_INV_PIN, RightLeftInv);

    Serial.println(RightLeft);
    // Serial.println(unRightLeftIn);
    // Serial.println(RightLeft);
    // Serial.println(RightLeft);

    
    
   }


}


// simple interrupt service routine
void calcForwardReverse()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(FORWARD_REVERSE_PIN) == HIGH)
  { 
    ulForwardReverseStart = micros();

  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unForwardReverseInShared = (uint16_t)(micros() - ulForwardReverseStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= FORWARD_REVERSE_FLAG;
  }

}

void calcRightLeft()
{
  if(digitalRead(RIGHT_LEFT_PIN) == HIGH)
  { 
    ulRightLeftStart = micros();
  }
  else
  {
    unRightLeftInShared = (uint16_t)(micros() - ulRightLeftStart);
    bUpdateFlagsShared |= RIGHT_LEFT_FLAG;
  }
}

void calcAux()
{
  if(digitalRead(AUX_IN_PIN) == HIGH)
  { 
    ulAuxStart = micros();
  }
  else
  {
    unAuxInShared = (uint16_t)(micros() - ulAuxStart);
    bUpdateFlagsShared |= AUX_FLAG;
  }
}
