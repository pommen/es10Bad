
/*

Om vi inte vet vart den är (första boot up) så går vi sakta upp. efter det så kaliberar vi längd.

soft nödstop (10 % av max)

en knapp för upp och ned (toggle)
led til fault condition



*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <PinButton.h>

//PINS:

const int fault = 11;
const int stopHighPin = 7;
const int stopLowPin = 6;
const int enable = 8;
const int toggleSwitch = 2;
const uint8_t PixelPin = A1; // make sure to set this to the correct pin, ignored for Esp8266
const int stepPin = 9;
const int dirPin = 10;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // Driver for stepPin, DirPin
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PixelPin, NEO_GRB + NEO_KHZ800);
PinButton myButton(toggleSwitch);

//LED colors:
// Define the color codes by name to make it easier to call later on
uint32_t red = pixels.Color(255, 0, 0);
uint32_t green = pixels.Color(0, 255, 0);
uint32_t yellow = pixels.Color(255, 100, 0);

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
int distance = 1000;
unsigned long timeSinceInPos = 0;
int runonce = 99;

//Motor settings:

int defaultSpeed = 20000; //20000
int defaultAcc = 3000;    //40000
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
    stepper.setMinPulseWidth(7);

    pinMode(fault, INPUT);
    pinMode(enable, OUTPUT);
    pinMode(stopHighPin, INPUT_PULLUP);
    pinMode(stopLowPin, INPUT_PULLUP);
    pinMode(toggleSwitch, INPUT_PULLUP);

    pixels.begin(); // This initializes the NeoPixel library.
    //pixels.setBrightness(36);
    timeSinceInPos = millis();
    Serial.println("Up and running! Waiting for button push to start cal ");
    while (digitalRead(toggleSwitch) != LOW)
    {
    }
    startCal();
    BootupBlink();
}

void loop()
{

    myButton.update(); // Get the updated value :
    if (millis() - timeSinceInPos > 5000)
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
        runMotor(0, -1000);
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
        Serial.println("enableToggle");
        enableDrive();
    }



    if (digitalRead(fault) == HIGH)
    {
        Serial.println("fault");
        faultState();
    }

    if (enableState == false && inPos == true)
        heartBeat();
    else if (enableState == true)
    {
        pixels.setPixelColor(0, yellow); // Moderately bright green color.
        pixels.show();                   // This sends the updated pixel color to the hardware.
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
    timeSinceInPos = millis();
    //se till att den ska röra sig långt, annars finns det chans att den deaccelererar mitt i körningen

    if (direction == 1)
    {
        stepper.moveTo(position);

        while (stepper.distanceToGo() != 0)
        {
            if (digitalRead(stopHighPin) == 0)
            { //sluta coasta när vi träffar sensorn
                Serial.println("Hit Upper sensor");
                faultState();
                break;
            }

            if (digitalRead(stopLowPin) == 0)
            { //sluta coasta när vi träffar sensorn
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
            { //sluta coasta när vi träffar sensorn
                faultState();

                break;
            }

            if (digitalRead(stopLowPin) == 0)
            { //sluta coasta när vi träffar sensorn
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

    stepper.setMaxSpeed(300);
    stepper.setAcceleration(200);
    stepper.moveTo(5000);

    while (stepper.distanceToGo() != 0)
    {
        if (digitalRead(stopHighPin) == 0)
        { //sluta coasta när vi träffar sensorn
            Serial.println("Hit Upper sensor");

            break;
        }
        stepper.run();
    }
    stepper.setCurrentPosition(0);

    stepper.moveTo(-300);

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