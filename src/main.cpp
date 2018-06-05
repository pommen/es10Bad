
/*

Om vi inte vet vart den är (första boot up) så går vi sakta upp. efter det så kaliberar vi längd.

en knapp för upp och ned (toggle)
led til fault condition



*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h> //RGB led
#include <PinButton.h>         //debounce för knappen

//PINS:

const int fault = 11;        // active low
const int stopHighPin = 7;   //Active LOW
const int stopLowPin = 6;    //active low
const int enable = 8;        //active LOW till motor drivare
const int toggleSwitch = 2;  //Active LOW
const uint8_t PixelPin = A1; //..pixel controller
const int stepPin = 9;       // till Motor drivare
const int dirPin = 10;       // till Motor drivare

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // Driver for stepPin, DirPin
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PixelPin, NEO_GRB + NEO_KHZ800);
PinButton myButton(toggleSwitch);

//LED colors:
// Define the color codes by name to make it easier to call later on
uint32_t red = pixels.Color(255, 0, 0);
uint32_t green = pixels.Color(0, 255, 0);
uint32_t yellow = pixels.Color(255, 100, 0);
uint32_t dark = pixels.Color(0, 0, 0);

//heartBeat Vars:
int beat = 0;
boolean hearbeatDown = false;
long heartBeatTimer = 0;
int heartBeatMax = 130;

//toggle & pos vars:
boolean toggle = false;
boolean toggleOld = false;
boolean atUpPos = false;
boolean atDownPos = false;
boolean inPos = false;
unsigned long timeSinceInPos = 0;

//Motor settings:
int defaultSpeed = 700; //20000
int defaultAcc = 800;  //40000
boolean enableState = false;

//Protos:
void heartBeat();
void BootupBlink();
void runMotor(int direction, int position);
void startCal();
void inputState();
void faultState();
void test();
void enableDrive();

void setup()
{
    Serial.begin(9600);
    stepper.setEnablePin(enable);
    stepper.setMaxSpeed(defaultSpeed);
    stepper.setAcceleration(defaultAcc);
    stepper.setMinPulseWidth(4);

    pinMode(fault, INPUT_PULLUP);
    pinMode(enable, OUTPUT);
    pinMode(stopHighPin, INPUT_PULLUP);
    pinMode(stopLowPin, INPUT_PULLUP);
    pinMode(toggleSwitch, INPUT_PULLUP);

    pixels.begin(); // This initializes the NeoPixel library.
    //pixels.setBrightness(36);
    timeSinceInPos = millis();
    pixels.setPixelColor(0, yellow); // Moderately bright yellow color.
    pixels.show();                   // This sends the updated pixel color to the hardware.
    Serial.println("Up and running! Waiting for button push to start cal ");
    while (digitalRead(toggleSwitch) != LOW) //väntar att man skal trycka på knappen innan vi kaliberar
    {
    }
    startCal();
    BootupBlink();
}

void loop()
{
     myButton.update();
    if (millis() - timeSinceInPos > 5000) //timeout, anti button Spam rutin. Kanske borde vara 20-30 sek?
    {
        /*     Serial.print("Inpos: ");
        Serial.println(inPos); */
        inPos = true;
    }
    else
        inPos = false;

    if (myButton.isSingleClick() && atUpPos == true && atDownPos == false && inPos == true)
    {
        inPos = false;
        Serial.println("Going Down");
        runMotor(0, -1780); //100step = 27mm
    }

    //     if (digitalRead(toggleSwitch) == LOW && atDownPos == true && atUpPos == false && inPos == true)
    if (myButton.isSingleClick() && atDownPos == true && atUpPos == false && inPos == true)

    {
        Serial.println("Going UP");
        inPos = false;
        runMotor(1, 0);
    }
    /*  else if (digitalRead(toggleSwitch) == LOW)
    {
        Serial.print("atDownPos: ");
        Serial.println(atDownPos);
        Serial.print("atUpPos: ");
        Serial.println(atUpPos);
    }
 */
    if (myButton.isLongClick())
    {
        Serial.print("enableToggle! Drive On: ");
        Serial.println(!enableState);
        enableDrive();
    }

    /* if (digitalRead(fault) == LOW)
    {
        Serial.println("fault");
        faultState();
    }
 */
    if (enableState == false && inPos == true)
        heartBeat();
    else if (enableState == true)
    {
        pixels.setPixelColor(0, yellow);
        pixels.show(); // This sends the updated pixel color to the hardware.
    }
     if (digitalRead(stopHighPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                Serial.println("Hit Upper sensor");
                faultState();
            }

            if (digitalRead(stopLowPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                Serial.println("Hit Lower sensor");
                faultState();
            }
}

void enableDrive()
{

    //enableState false = drive on, enableState true = drive off.
    digitalWrite(enable, enableState);
    enableState = !enableState;
    if (enableState == false)
        startCal();
}

void runMotor(int direction, int position)
{
    //up = direction 1
    // down = 0

    if (direction == 1) //kör upp ur badet
    {
        stepper.moveTo(position);

        while (stepper.distanceToGo() != 0)
        {
            if (digitalRead(stopHighPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                Serial.println("Hit Upper sensor");
                faultState();
                break;
            }

            if (digitalRead(stopLowPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                Serial.println("Hit Lower sensor");
                faultState();
                break;
            }

            //Serial.println(stepper.distanceToGo());
            stepper.run();
        }
        atUpPos = true;
        atDownPos = false;
    }

    if (direction == 0)
    {
        stepper.moveTo(position);

        while (stepper.distanceToGo() != 0)
        {

            if (digitalRead(stopHighPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                faultState();

                break;
            }

            if (digitalRead(stopLowPin) == 0)
            { //träffar vi denna så har vi gått för långt/något har lossnat
                faultState();

                break;
            }

            //Serial.println(stepper.distanceToGo());
            stepper.run();
        }
        atUpPos = false;
        atDownPos = true;
    }
    Serial.print("At position: ");
    Serial.println(stepper.currentPosition());
    timeSinceInPos = millis();
}

void BootupBlink() //bootup last thing before loop()
{

    for (size_t i = 0; i < 50; i++)
    {
        pixels.setPixelColor(0, pixels.Color(-i, i, -i));
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(3);
    }
    for (size_t i = 0; i < 50; i++)
    {
        pixels.setPixelColor(0, pixels.Color(-i, -i, i));
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(3);
    }
    for (size_t i = 0; i < 50; i++)
    {
        pixels.setPixelColor(0, pixels.Color(i, -i, -i));
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(3);
    }
    pixels.setPixelColor(0, dark);
}
void heartBeat() //Im alive!
{

    if (millis() - heartBeatTimer > 20)
    {
        pixels.setPixelColor(0, pixels.Color(0, beat, 0)); // Moderately bright green color.
        pixels.show();                                     // This sends the updated pixel color to the hardware.
        if (beat > heartBeatMax && hearbeatDown == false)
            hearbeatDown = true;
        if (beat < 10 && hearbeatDown == true)
            hearbeatDown = false;

        if (hearbeatDown == false)
        {
            beat = beat + 4;
        }
        else
            beat = beat - 4;

        heartBeatTimer = millis();
    }
}
void faultState()
{
    Serial.print("Faultstate!");
    digitalWrite(enable, LOW);
    while (1)
    {
        pixels.setPixelColor(0, red); // Moderately bright green color.
        pixels.show();                // This sends the updated pixel color to the hardware.
        delay(500);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Moderately bright green color.
        pixels.show();                                  // This sends the updated pixel color to the hardware.
        delay(500);
    }
}
void startCal()
{
    stepper.setMaxSpeed(150);
    stepper.setAcceleration(500);
    stepper.moveTo(5000);

    //stepper.distanceToGo() != 0
    while (digitalRead(stopHighPin) != 0)
    {
        /*    if ( == 0)
        { //sluta coasta när vi träffar sensorn
            Serial.println("Hit Upper sensor");

            break;
        } */
        stepper.run();
    }
    delay(200);
    stepper.setCurrentPosition(0);

    stepper.moveTo(-150);

    while (stepper.distanceToGo() != 0)
    {
        stepper.run();
    }
    atUpPos = true;
    atDownPos = false;
    stepper.setCurrentPosition(0);
    Serial.println("Calibrated!");
    stepper.setMaxSpeed(defaultSpeed);
    stepper.setAcceleration(defaultAcc);
    timeSinceInPos = millis();
}

void inputState()
{
    Serial.print("Toggle: ");
    Serial.print(digitalRead(toggleSwitch));
    Serial.print(" Upper: ");
    Serial.print(digitalRead(stopHighPin));
    Serial.print(" Lower: ");
    Serial.println(digitalRead(stopLowPin));

    delay(100);
}
/* void test()
{
    stepper.distanceToGo(100);
    while (stepper.distanceToGo() > 0)
    {
        stepper.run();
    }
} */