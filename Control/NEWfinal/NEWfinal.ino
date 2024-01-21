#include <Regexp.h>
#include <time.h>
// Pin definitions for software serial
const int motorDriverRxPin = 4;     // RX pin for motor driver
const int motorDriverTxPin = 5;     // TX pin for motor driver


unsigned long time1;
unsigned long time2;
unsigned long telapsed;

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

void setup() {

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(57600);

  Serial1.begin(115200); 
  // motorDriverSerial.begin(115200);


  node.begin(1, Serial1);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  node.writeSingleRegister(0x200e, 0x08);     // Enable

  node.writeSingleRegister(0x200e, 0x10);     // Start

  node.writeSingleRegister(0x200e, 0x08);     // Enable
  node.writeSingleRegister(0x200e, 0x10);
}

void control(float error, int velocity){
  int v_l = velocity + error;
  int v_r = velocity - error;
  
  // int v_l = 0;
  // int v_r = 0;
  // if(error >= 0){
  //   // Turn Right
  //   v_l = map(error, 0, 90, 40, 90);
  //   v_r = map(error, 0, 90, 40, 30);
  // }else if(error < 0){
  //   v_l = map(error, 0, -90, 40, 30);
  //   v_r = map(error, 0, -90, 40, 90);
  // }
  // Serial.println("v_l: " + String(v_l / 2));
  // Serial.println("v_r: " + String(v_r / 2));

  
  // node.clearTransmitBuffer();
  node.setTransmitBuffer(0, -v_l / 2);
  node.setTransmitBuffer(1, v_r / 2);
  node.writeMultipleRegisters(0x2088, 2);
  // delay(time * 1000);
}

void loop() {
  // time1 = millis();
  if (Serial.available() > 0) {
    char recieved = Serial.read();
    inData += recieved;
    if (recieved == '\n') {
      char parameter = inData[0];
      String value = inData.substring(1, inData.length());
      int angle = value.toInt();
      // Serial.println(parameter);
      // Serial.println(angle);
      inData = "";
      switch(parameter){
        case 'E':
          control(angle, 90);
          // time2 = millis();
          // telapsed = time2 - time1;
          // Serial.println("d_t" + String(telapsed));
        break;
        case 'S':
          // errorToMove(angle * Pi / 180);
          // control(angle, 0);
          // node.clearTransmitBuffer();
          node.setTransmitBuffer(0, 0);
          node.setTransmitBuffer(1, 0);
          node.writeMultipleRegisters(0x2088, 2);
    
        break;
        default:
        break;
      }
    // delay(200);
    }
  }
  
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
  delay(100);

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
