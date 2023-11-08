#include <Regexp.h>
#include <SoftwareSerial.h>

// Pin definitions for software serial
const int motorDriverRxPin = 4;     // RX pin for motor driver
const int motorDriverTxPin = 5;     // TX pin for motor driver

// Create a software serial object
SoftwareSerial motorDriverSerial(motorDriverRxPin, motorDriverTxPin);

String inData;
void match_callback(const char *match,          // matching string (not null-terminated)
                    const unsigned int length,  // length of matching string
                    const MatchState &ms)       // MatchState in use (to get captures)
{
  char cap[10];

  Serial.print("Matched: ");
  Serial.write((byte *)match, length);
  Serial.println();

  for (byte i = 0; i < ms.level; i++) {
    Serial.print("Capture ");
    Serial.print(i, DEC);
    Serial.print(" = ");
    ms.GetCapture(cap, i);
    Serial.println(cap);
  }  // end of for each capture

}  // end of match_callback



#include <ModbusMaster.h>     // Arduino library for communicating with Modbus slaves over RS485

#define MAX485_DE      3
#define MAX485_RE_NEG  2
#define LED 13

ModbusMaster node;

const double Pi = 3.141592;
double radius { 6.3 };        // Radius of Wheels
double D { 40 };              // distance between two wheels

uint16_t result;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


// void setup() {
//   pinMode(MAX485_RE_NEG, OUTPUT);
//   pinMode(MAX485_DE, OUTPUT);
  
//   digitalWrite(MAX485_RE_NEG, 0);
//   digitalWrite(MAX485_DE, 0);
//   Serial.begin(115200);
// }
void setup() {

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(57600);
  motorDriverSerial.begin(115200);


  node.begin(1, motorDriverSerial);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  node.writeSingleRegister(0x200e, 0x08);     // Enable

  node.writeSingleRegister(0x200e, 0x10);     // Start
}

void control(float error, uint16_t velocity){
 

  int v_l = 0;
  int v_r = 0;
  
  if(error >= 0){
    v_l = map(error, 0, 90, 20, 50);
    v_r = map(error, 0, 90, 20, -50);
  }else if(error < 0){
    v_l = map(error, 0, -90, 50, -70);
    v_r = map(error, 0, -90, 50, 70);
  }
  Serial.println("v_l: " + String(v_l));
  Serial.println("v_r: " + String(v_r));

  // int time = 0.1;

  // node.clearTransmitBuffer();
  node.setTransmitBuffer(0, -v_l);
  node.setTransmitBuffer(1, v_r);
  node.writeMultipleRegisters(0x2088, 2);
  // delay(time * 1000);
}

void loop() {
  if (Serial.available() > 0) {
    char recieved = Serial.read();

    inData += recieved;
    if (recieved == '\n') {
      char parameter = inData[0];
      // String value = inData.substring(1, inData.length());
      int angle = inData.toInt();
      // Serial.println(parameter);
      // Serial.println(angle * Pi / 180);
      control(angle, 20);
      // inData = "";

      // switch(parameter){
      //   case 'E':
      //     Serial.println(parameter);
      //     // errorToMove(angle * Pi / 180);
          

      //   break;
      //   default:
      //   break;
      // }
    delay(10);
    }
  }
}

void errorToMove(float error){
  node.writeSingleRegister(0x200e, 0x08);     // Enable

  node.writeSingleRegister(0x200e, 0x10);
  int direction = (error > 0) ? 1 : 0;
  Serial.println(direction);
  Serial.println((error > 0) ? "Forward" : "Reverse");
  float angle = abs(error);
  Serial.println(angle);
  int velocity = 50; 
  if(angle < Pi/2 && angle >= Pi/3){
    TurnInPlace(velocity, angle, direction);
    Serial.println("State 1: Turn");
  }else if(angle < Pi/3 && angle >= Pi/36 ){
    OneWheelTurn(velocity, angle, direction);
    Serial.println("State 2: OneWheel");

  }else if(angle < Pi/36 && angle >= 0 ){
    MoveStraight(velocity, 60);
    Serial.println("State 3: Straight");

  }
  node.writeSingleRegister(0x200e, 0x07);
}

double RPMtoMPS (double velocity)
{
  return ( (2.0 * Pi * radius * velocity ) / 60.0);
}

void MoveStraight(uint16_t velocity, uint16_t distance )
/* velocity (rpm) , distance (cm)
This function Moves forward until that distance but does not end up with stop command
*/
{
  double time = (double)distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, -velocity);
  node.setTransmitBuffer(1, velocity);
  node.writeMultipleRegisters(0x2088, 2);
  delay(time * 1000);
}

void TurnInPlace(uint16_t velocity, double angle, bool direction)
// direction 0 turns right and 1 turns left, angle radian
{
  double distance { angle * (D / 2) };
  double time = distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  if ( direction == 0)
  {
    node.setTransmitBuffer(0, -velocity);
    node.setTransmitBuffer(1, -velocity);
  }
  else
  {
    node.setTransmitBuffer(0, velocity);
    node.setTransmitBuffer(1, velocity);
  }
  node.writeMultipleRegisters(0x2088, 2);

  delay(time * 1000);
}

void OneWheelTurn(uint16_t velocity, double angle, bool direction)
//  velocity RPM , angle radian, direction 0 turns right and 1 turns left
{
  double distance { D * angle };
  double time = distance / abs(RPMtoMPS((double)velocity));
  node.clearTransmitBuffer();
  if(direction == 0)
  {
    node.setTransmitBuffer(0, -velocity);
    node.setTransmitBuffer(1, 0);
  }
  else
  {
    node.setTransmitBuffer(0, 0);
    node.setTransmitBuffer(1, velocity);
  }
  node.writeMultipleRegisters(0x2088, 2);

  delay(time * 1.07 * 1000 );

}

void wait(unsigned long time)
{
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, 0);
  node.setTransmitBuffer(1, 0);
  node.writeMultipleRegisters(0x2088, 2);
  delay(time);
}
