#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <functional>
#include "switch.h"
#include "UpnpBroadcastResponder.h"
#include "CallbackFunction.h"


// Site specfic variables
const char* ssid = "Guest";
const char* password = "specialphonenumber";

const int dooropenupperthreshold = 60; //cm.  Readings over this value indicate door is closed - adjust to suit door configuartion
const int dooropenlowerthreshold = 10; //cm.  Readings under this value are discarded as invalid
const int RelayPulseLength = 4000;    //Length of relay pulse in ms (has RC timeconstant on the relay drive to reduce startup pulse problem.

// prototypes
boolean connectWifi();

//on/off callbacks
bool GarageDoorOpenOn();
bool GarageDoorOpenOff();
bool GarageDoorCloseOn();
bool GarageDoorCloseOff();
bool GarageDoorRebootOn();
bool GarageDoorRebootOff();

boolean wifiConnected = false;

UpnpBroadcastResponder upnpBroadcastResponder;

Switch *Garagedooropen = NULL;
Switch *Garagedoorclose = NULL;
Switch *Garagedoorboot = NULL;

bool isGarageDoorOpenOn = false;
bool isGarageDoorCloseOn = false;
bool isGarageDoorBootOn = false;
bool GarageDoorState = false;
bool PrevGarageDoorState = false;
bool PulseRelaySemaphore = false;

const int GarageDoorRelay = 0;  // GPIO0 pin.
const int GPIO2 = 2;  // GPIO2 pin.

long duration, distance; // Duration used to calculate distance

const int echoPin = 3;  // 3 is the RX pin
const int trigPin = 1; // 1 is the TX Pin

/*  WIRING DETAILS
 *  CPU TXD connects to the ultrasonic Trigger pin  
 *  CPU RXD connects to the ultrasonic Echo pin
 */

void setup()
{
  pinMode(GarageDoorRelay, OUTPUT);
  pinMode(GPIO2, OUTPUT); // to start with, it changes later...

  Serial.begin(9600);

  Serial.println("Booting...");
  delay(2000);
  Serial.println("flashing LED on GPIO2...");
  //flash fast a few times to indicate CPU is booting
  digitalWrite(GPIO2, LOW);
  delay(100);
  digitalWrite(GPIO2, HIGH);
  delay(100);
  digitalWrite(GPIO2, LOW);
  delay(100);
  digitalWrite(GPIO2, HIGH);
  delay(100);
  digitalWrite(GPIO2, LOW);
  delay(100);
  digitalWrite(GPIO2, HIGH);

  Serial.println("Delaying a bit...");
  delay(2000);

  // Initialise wifi connection
  wifiConnected = connectWifi();

  if (wifiConnected) {
    Serial.println("flashing slow to indicate wifi connected...");
    //flash slow a few times to indicate wifi connected OK
    digitalWrite(GPIO2, LOW);
    delay(1000);
    digitalWrite(GPIO2, HIGH);
    delay(1000);
    digitalWrite(GPIO2, LOW);
    delay(1000);
    digitalWrite(GPIO2, HIGH);
    delay(1000);
    digitalWrite(GPIO2, LOW);
    delay(1000);
    digitalWrite(GPIO2, HIGH);

    Serial.println("starting upnp responder");
    upnpBroadcastResponder.beginUdpMulticast();

    // Define your switches here. Max 10
    // Format: Alexa invocation name, local port no, on callback, off callback
    Garagedooropen = new Switch("Garage door open", 90, GarageDoorOpenOn, GarageDoorOpenOff);
    Garagedoorclose = new Switch("Garage door close", 91, GarageDoorCloseOn, GarageDoorCloseOff);
    Garagedoorboot = new Switch("Garage door controller reboot", 92, GarageDoorBootOn, GarageDoorBootOff);

    Serial.println("Adding switches upnp broadcast responder");
    upnpBroadcastResponder.addDevice(*Garagedooropen);
    upnpBroadcastResponder.addDevice(*Garagedoorclose);
    upnpBroadcastResponder.addDevice(*Garagedoorboot);

  }

  digitalWrite(GarageDoorRelay, LOW); // turn off relay
  digitalWrite(GPIO2, HIGH); // turn off LED

  Serial.println("Making RX pin into an INPUT"); // used to detect Garage door current state
  //GPIO 3 (RX) swap the pin to a GPIO.

  pinMode(echoPin, FUNCTION_3);
  pinMode(echoPin, INPUT);


  PrevGarageDoorState = GarageDoorState; // edge detection of Garage door state
}

void loop()
{

  delay(1000); // Main loop timer - sets frequency of measuring door distance

  distance = 1; //dummy value - will be overwritten by actual measurement each pass. Must be < lowerthreshold

  while (distance < dooropenlowerthreshold ) { // if its less than min, throw that reading away as invalid

    measuredistance();

  }
  //Serial.println(duration);
  Serial.println(distance);

  if (distance < dooropenupperthreshold) {
    GarageDoorState = LOW;
    //  Serial.println("Door Open");
  }
  else {
    GarageDoorState = HIGH;
    //  Serial.println("Door Closed");
  }


  //GarageDoorState = digitalRead(RXPin); //either GPIO 2 or RX Pin depending on variable used...
  delay(100);
  if (GarageDoorState == LOW) {
    if (PrevGarageDoorState == HIGH) {
      //Serial.println("GarageDoor State has just opened (Logic LOW)");
    }
  }

  if (GarageDoorState == HIGH) {
    if (PrevGarageDoorState == LOW) {
      //Serial.println("GarageDoor State has just closed (Logic HIGH)");
    }
  }

  PrevGarageDoorState = GarageDoorState;   // remember prev state for next pass

  if (wifiConnected) {

    upnpBroadcastResponder.serverLoop();

    Garagedoorboot->serverLoop();
    Garagedoorclose->serverLoop();
    Garagedooropen->serverLoop();
  }

  if (PulseRelaySemaphore == HIGH) {

    //Serial.println("XXX Pulsing Relay on ...");
    digitalWrite(GarageDoorRelay, HIGH); // turn on relay
    delay(RelayPulseLength);    ;
    //Serial.println("XXX Pulsing Relay off again ...");
    digitalWrite(GarageDoorRelay, LOW); // turn off relay
    // this code here, as a delay of >5 sec in the routines that run
    // as each control activates makes alexa think the devices are unresponsive
    // in this way, the alexa response is sent quickly, and the slow relay pulse
    // is done outside that loop

    PulseRelaySemaphore = LOW;
  }
}

bool GarageDoorOpenOn() {
  //Serial.println("Request to Open door received ...");

  if (GarageDoorState == HIGH) { // only pulse relay if door is currently closed
    //Serial.println("Door is closed - pulsing relay to open it");
    PulseRelaySemaphore = HIGH; // set relay semaphore

  }
  else {
    //Serial.println("Door is already open - not pulsing relay!");
  }

  isGarageDoorOpenOn = false;
  return GarageDoorState;
}

bool GarageDoorOpenOff() {

  //Serial.println("Request to Open door received SW#1 Off ...");

  if (GarageDoorState == HIGH) { // only pulse relay if door is currently closed
    //Serial.println("Door is closed - pulsing relay to open it");
    PulseRelaySemaphore = HIGH; // set relay semaphore

  }
  else {
    //Serial.println("Door is already open - not pulsing relay!");
  }

  isGarageDoorOpenOn = false;
  return GarageDoorState;
}

bool GarageDoorCloseOn() {
  // Serial.println("Request to Close door received ...");

  if (GarageDoorState == LOW) { // only pulse relay if door is currently open
    // Serial.println("Door is open - pulsing relay to close it");
    PulseRelaySemaphore = HIGH; // set relay semaphore

  }
  else {
    // Serial.println("Door is already closed, not pulsing relay...");
  }

  isGarageDoorCloseOn = false;
  return GarageDoorState;
}

bool GarageDoorCloseOff() {

  //Serial.println("Request to Close door received ..");

  if (GarageDoorState == LOW) { // only pulse relay if door is currently open
    //Serial.println("Door is open - pulsing relay to close it");
    PulseRelaySemaphore = HIGH; // set relay semaphore

  }
  else {
    //Serial.println("Door is already closed, not pulsing relay...");
  }

  isGarageDoorCloseOn = false;
  return GarageDoorState;
}




bool GarageDoorBootOn() {
  Serial.println("Request to reboot door controller received... ");

  //ESP.restart(); hangs everytime..
  ESP.reset(); // works sometimes...

  isGarageDoorBootOn = false;
  return isGarageDoorBootOn;
}

bool GarageDoorBootOff() {

  //Serial.println("Request to reboot Garage door controller received (SW#3 Off)");
  // Never called...


  isGarageDoorBootOn = false;
  return GarageDoorState;
}



// connect to wifi – returns true if successful or false if not
boolean connectWifi() {
  boolean state = true;
  int i = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi Network");

  // Wait for connection
  Serial.print("Connecting ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.print(".");
    if (i > 10) {
      state = false;
      break;
    }
    i++;
  }

  if (state) {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("");
    Serial.println("Connection failed. Bugger");
  }

  return state;
}

void measuredistance() {
  delay(100); //pause to stabilise

  /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it.
    The ESP-01 running on a USB adaptor gets confused easily by power supply noise
    with this code. Beware!
    especally important to run the ultrasonic sensor on its own PSU - it gets
    very septic running on the USB power as well. A grunty capacitor across
    its PSU pins helps as well.
    YMMV
  */
  Serial.println("~"); // this will trigger the ultrasonic to take a measurement
  
  duration = pulseIn(echoPin, HIGH);
  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration / 58.2;

}
