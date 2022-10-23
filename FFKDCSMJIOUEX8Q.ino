/**************************************************************************

  ACOUSTIC RADAR SENDER 1
  acoustic_radar_sender_1.ino
  Copyright (C) 2018 by lingib
  https://www.instructables.com/member/lingib/instructables/
  Last update 12 June 2018

  ----------
  Notes
  ----------
  (1) This routine sends ultrasound "Azimuth, Distance1, Distance2, Direction"
      information to a graphics terminal on receipt of a send command.

  (2) Globals variables start with a capital letter

  ----------
  Copyright
  ----------
  This code is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published
  by the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License. If
  not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// ======================
// globals
// ======================

// ----- serial port
#define Baud_rate 115200          //communication speed


// ----- serial input
char Char;

// ----- micro-switch
#define Micro_switch 4
bool Switch_present = true;       //no micro-switch = false

// ----- HC-SR04 / HY-SRF05 ultrasonic transducer
#define Trig 5
#define Echo 6
#define RawEcho 7                 //extra wire connected to pin 10 (see text)

// ----- motor controller definitions
/*
   Connect your motor controller pins IN1..IN4 to the following Arduino pins.
   The Arduino "talks" directly to controller pins IN1..IN4 via PORTB.
*/
#define IN4  8
#define IN3  9
#define IN2  10
#define IN1  11

// ----- motor definitions
/*
    The 28BJY-48 5V DC motor has a "stride Angle" of 5.625/64 degrees per step.
    One complete revolution therefore requires 360/5.625*64 equals 4096 steps.

    This motor may be run using one of three possible modes:
    1. Wave-stepping: lowest torque, max speed, coarse movement
    2. Full-stepping: highest torque, max speed, coarse movement
    3. Half-stepping: medium torque, half speed, smooth movement

    From experiment "full-stepping" (which has the most torque) requires a minimum
    delay of 2mS for reliable starting, whereas "half-stepping" only requires 1mS.
    As a result the rotation speeds are the same. Since there is no speed advantage
    "half-stepping" has been chosen for smoothness of rotation and reliable starting.

    The required "half-stepping" motor pattern to achieve this is shown below.
*/
// ----- motor pattern
byte Motor[8] =                  //half-stepping
{ B00001000,
  B00001100,
  B00000100,
  B00000110,
  B00000010,
  B00000011,
  B00000001,
  B00001001
};

int Index = 0;                  //Motor[] array index
int Step_counter = 0;           //180 degrees requires 2048 steps
unsigned long Delay = 2;        //give motor shaft time to move
byte Pattern;                   //Motor[] pattern

// ----- acoustic "radar" display data
int Azimuth = 0;                //Azimuth (PI/128 radians) measured CCW from reference
int Distance1 = 0;
int Distance2 = 0;
int Direction = 0;              //counter-clockwise=0, clockwise=1

unsigned long
Speed_of_rotation = 30;         //controls beam rotation: 1 = fastest

// ======================
// setup
// ======================
void setup()
{
  // ----- configure serial port
  Serial.begin(Baud_rate);

  // ----- configure micro-switch
  pinMode(Micro_switch, INPUT_PULLUP);  //"wire-OR" normally HIGH

  // ----- configure arduino pinouts
  pinMode(Echo, INPUT);               //make Echo pin an input
  pinMode(RawEcho, INPUT);            //make RawEcho pin an input
  pinMode(Trig, OUTPUT);              //set Trig pin LOW
  digitalWrite(Trig, LOW);

  // ----- configure stepper motor
  Pattern = DDRB;                       // get PORTB data directions
  Pattern = Pattern | B00001111;        // preserve MSN data direction &
  DDRB = Pattern;                       // make pins 8,9,10,11 outputs

  // ----- rotate beam to start-up position
  if (Switch_present)
  {
    home();
  }

  // ----- attach the graphics display
  connect_to_display();                 //connect to the display
}

// ======================
// loop
// ======================
void loop()
{
  // ----- has the display asked for data
  if (Serial.available() > 0)
  {
    Char = Serial.read();               // read character

    // ----- send data to display whenever a send character ('S') is received
    if (Char == 'S')
    {
      // ----- measure distances
      measure();

      // ----- rotate beam to next ping position
      rotate();

      // ----- send the results to the display
      Serial.print(Azimuth);
      Serial.print(',');
      Serial.print(Distance1);
      Serial.print(',');
      Serial.print(Distance2);
      Serial.print(',');
      Serial.println(Direction);

      delay(Speed_of_rotation);         //slows rotational speed
    }
  }
}

// ===============================
// connect to graphics display
// ===============================
void connect_to_display()
{
  while (Serial.available() <= 0)
  {
    // ----- keep sending synch ('S') until the display responds
    Serial.println("S");
    delay(250);
  }
}

// ===============================
// measure distances
// ===============================
void measure()
{
  // ----- locals
  unsigned long start_time;           //microseconds
  unsigned long finish_time;          //microseconds
  unsigned long time_taken;           //microseconds
  unsigned long timeout;              //microseconds
  unsigned long pause;                //microseconds
  boolean flag;

  // ----- generate 10uS start pulse
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  // ----- wait for pulse(s) to be sent
  while (!digitalRead(Echo));                 //wait for high
  start_time = micros();

  // ----- set timeout radius
  timeout = start_time + 12000;               //set timeout radius to 2 meters

  // ----- measure first object distance
  flag = false;
  while (!flag)
  {
    if (!digitalRead(Echo)) flag = true;      //exit loop if object detected
    if (timeout < micros()) flag = true;      //exit loop if timeout exceeded
  }
  finish_time = micros();

  // ----- calculate first object distance(cm)
  time_taken = finish_time - start_time;
  Distance1 = ((float)time_taken) / 59;

  // ----- wait for first object echo to finish
  pause = finish_time + 1000;                 //1000uS means 17cm closest object spacing
  while (pause > micros());                   //wait 1000uS

  // ----- measure second object distance
  flag = false;
  while (!flag)                               //wait for high
  {
    if (digitalRead(RawEcho)) flag = true;    //exit loop if object dectected
    if (timeout < micros()) flag = true;      //exit loop if timeout exceeded
  }
  finish_time = micros();

  // ----- calculate second object distance (cm)
  time_taken = finish_time - start_time;
  Distance2 = ((float)time_taken) / 59;
}

// ===============================
// rotate motor to next ping position
// ===============================
void rotate()
{
  // ----- counter-clockwise scan
  if (Direction == 0)
  {
    for (int i = 0; i < 8; i++)
    {
      // ----- rotate motor to next ping position
      Index = Step_counter % 8;                 //calculate array index
      Pattern = PORTB;                          //get current motor pattern
      Pattern = Pattern & B11110000;            //preserve MSN
      Pattern = Pattern | Motor[Index];         //create new motor pattern
      PORTB = Pattern;                          //send new pattern to motor
      Step_counter++;
      delay(Delay);                             //controls motor speed (fastest=1)
    }

    // ----- loop control
    Azimuth++;
    if (Azimuth > 256)
    {
      Azimuth = 256;
      Direction = 1;
      Step_counter = 2048;
    }
  }
  else
  {
    // ----- clockwise scan
    for (int i = 0; i < 8; i++)
    {
      // ----- rotate motor to next ping position
      Index = Step_counter % 8;                 //calculate array index
      Pattern = PORTB;                          //get current motor pattern
      Pattern = Pattern & B11110000;            //preserve MSN
      Pattern = Pattern | Motor[Index];         //create new motor pattern
      PORTB = Pattern;                          //send new pattern to motor
      Step_counter--;
      delay(Delay);                             //controls motor speed (fastest=1)
    }

    // ----- loop control
    Azimuth--;                                  //decrement Azimuth every 8 steps
    if (Azimuth < 0)
    {
      Azimuth = 0;
      Direction = 0;
      Step_counter = 0;
    }
  }
}

// ===============================
// find zero position for beam
// ===============================
void home()
{
  // ----- rotate clockwise until limit switch operates
  Step_counter = 2048;
  while (digitalRead(Micro_switch))
  {
    Index = Step_counter % 8;                 //calculate array index
    Pattern = PORTB;                          //get current motor pattern
    Pattern = Pattern & B11110000;            //preserve MSN
    Pattern = Pattern | Motor[Index];         //create new motor pattern
    PORTB = Pattern;                          //send new pattern to motor
    Step_counter--;
    delay(Delay);                             //controls motor speed (fastest=1)
  }

  // ----- back off slightly
  /*
     Keep clear of limit switch during normal scans
  */
  Step_counter = 0;
  for (int i = 0; i < 250; i++)
  {
    Index = Step_counter % 8;                 //calculate array index
    Pattern = PORTB;                          //get current motor pattern
    Pattern = Pattern & B11110000;            //preserve MSN
    Pattern = Pattern | Motor[Index];         //create new motor pattern
    PORTB = Pattern;                          //send new pattern to motor
    Step_counter++;
    delay(Delay);                             //controls motor speed (fastest=1)
  }

  // ----- reset the step counter
  Step_counter = 0;
}





