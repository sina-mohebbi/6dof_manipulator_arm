/*
 *  Test program for Arduino RS422/RS485 Shield 
 *  Version 1.0
 *  Copyright (C) 2018  Hartmut Wendt  www.zihatec.de
 *  
 *  (based on sources of https://github.com/angeloc/simplemodbusng)
 *  
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/  

#include <SimpleModbusSlave.h>
#include <Stepper.h>
#define  Main_Motor_in1  0 // motor 1 IN1
#define  Main_Motor_in2  1 // motor 1 IN2
#define  Main_Motor_en  3 // motor 1 Enable
#define  Motor2_in1  2 // motor 2 IN1
#define  Motor2_in2  7 // motor 2 IN2
#define  Motor2_en  5 // motor 2 Enable
#define  Motor3_in1  14 // motor 3 IN1
#define  Motor3_in2  15 // motor 3 IN2
#define  Motor3_en  6 // motor 3 Enable
#define  StepMo_In1  10 // Stepper Motor In 1
#define  StepMo_In2  11 // Stepper Motor In 2
#define  StepMo_In3  12 // Stepper Motor In 3
#define  StepMo_In4  13 // Stepper Motor In 4
#define  Main_Angle_Pulse  16 // Main Angle Pulse Input
#define  Angle2_Pulse  17 // Angle 2 Pulse Input
#define  Angle3_Pulse  18 // Angle 3 Pulse Input
#define  StepMo_Rst  8 // Stepper Motor Reset Position
#define  Main_Angle_rst  9 // Main Angle Reset Position
#define  Angle2_Rst  19 // Angle 2 Reset Position
#define  Angle3_Rst  4 // Angle 3 Reset Position
#define  servo1      23

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(512, StepMo_In1, StepMo_In2, StepMo_In3, StepMo_In4);

/* This example code has 9 holding registers. 6 analogue inputs, 1 button, 1 digital output
   and 1 register to indicate errors encountered since started.
   Function 5 (write single coil) is not implemented so I'm using a whole register
   and function 16 to set the onboard Led on the Atmega328P.
   
   The modbus_update() method updates the holdingRegs register array and checks communication.

   Note:  
   The Arduino serial ring buffer is 128 bytes or 64 registers.
   Most of the time you will connect the arduino to a master via serial
   using a MAX485 or similar.
 
   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
   In a function 16 request the master will attempt to write to your 
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   118 bytes or 59 registers.
 
   Using the FTDI USB to Serial converter the maximum bytes you can send is limited 
   to its internal buffer which is 60 bytes or 30 unsigned int registers. 
 
   Thus:
 
   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 54 bytes or 27 registers.
 
   In a function 16 request the master will attempt to write to your 
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   50 bytes or 25 registers.
 
   Since it is assumed that you will mostly use the Arduino to connect to a 
   master without using a USB to Serial converter the internal buffer is set
   the same as the Arduino Serial ring buffer which is 128 bytes.
*/
 

// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  Main_Motor_CMD,//Main motor command
  Motor2_CMD,//Angle motor 2 command with gearbox
  Motor3_CMD,//Angle motor 3 command
  StepperMotor_CMD,// Stepper motor command   
  
  Main_Motor_Pos_CMD,//Main motor angle command
  Motor2_Pos_CMD,//Angle motor 2  angle command with gearbox
  Motor3_Pos_CMD,//Angle motor 3  angle command
  StepperMotor_Pos_CMD,// Stepper motor command    
  Angle1,// Main Angle of manipullator
  Angle2,// Angle of first forearm
  Angle3,// Angle of second forearm
  Position4,//Position fo gripper
  StepperMotor_Pos,//Position of Stepper motor item
  Man_Auto,//Manual=0/Auto=1 type of manippulator control
  Main_Motor_Status,// Run=1, Stop=0,
  Motor2_Status,// Run=1, Stop=0,
  Motor3_Status,// Run=1, Stop=0,
  StepperMotor_Status,// Run=1, Stop=0,
  Loc_Rem,// Local=0, Remote=1
  servo1_CMD,
  
  TOTAL_ERRORS,
  // leave this one
  TOTAL_REGS_SIZE 
  // total number of registers for function 3 and 16 share the same register array
};

unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

 
int Step_Pos=0;

int Main_Angle=0;
bool Main_Pulse=0;

int step_angle=6;

int Motor_2_Angle=0;
bool Angle_2_Pulse=0;

int Motor_3_Angle=0;
bool Angle_3_Pulse=0;


void setup()
{
  /* parameters(long baudrate, 
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned char low latency)
                
     The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
     to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx.
     Low latency delays makes the implementation non-standard
     but practically it works with all major modbus master implementations.
  */
  
  modbus_configure(9600, 1, 6, TOTAL_REGS_SIZE, 0);
  
    // set the speed of the motor to 30 RPMs
  stepper.setSpeed(30);
 
  pinMode(Main_Motor_in1, OUTPUT);
  pinMode(Main_Motor_in2, OUTPUT);
  pinMode(Main_Motor_en, OUTPUT);  
  pinMode(Motor2_in1, OUTPUT);
  pinMode(Motor2_in2, OUTPUT);
  pinMode(Motor2_en, OUTPUT);  
  pinMode(Motor3_in1, OUTPUT);
  pinMode(Motor3_in2, OUTPUT);
  pinMode(Motor3_en, OUTPUT);   
  pinMode(Main_Angle_Pulse, INPUT); 
  pinMode(Angle2_Pulse, INPUT);
  pinMode(Angle3_Pulse, INPUT);
  pinMode(StepMo_Rst, INPUT);
  pinMode(Main_Angle_rst, INPUT);  
  pinMode(Angle2_Rst, INPUT);
  pinMode(Angle3_Rst, INPUT);
  pinMode( servo1 ,INPUT);
 /* 
  while (not(digitalRead(StepMo_Rst))){  // Stepper Motor resetting
      stepper.step(-5);   
  }
  Step_Pos=0;
  holdingRegs[StepperMotor_Pos]=Step_Pos;
  
  while (not(digitalRead(Angle2_Rst))){  // Motor 2 resetting
        digitalWrite(Motor2_in1, LOW); 
        digitalWrite(Motor2_in2, HIGH);
        digitalWrite(Motor2_en, HIGH);  
  }
  digitalWrite(Motor2_en, LOW); 
  Motor_2_Angle=0;
  holdingRegs[Angle2]=Motor_2_Angle;
  
  while (not(digitalRead(Angle3_Rst))){  // Motor 3 resetting
        digitalWrite(Motor3_in1, LOW); 
        digitalWrite(Motor3_in2, HIGH);
        digitalWrite(Motor3_en, HIGH);  
  }
  digitalWrite(Motor3_en, LOW); 
  Motor_3_Angle=0;
  holdingRegs[Angle3]=Motor_3_Angle;
  
  
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
  */
  }


// change this to the number of steps on your motor
//#define Steps 0
int Steps=0; 


int DC_Motor_Manual(int Motor_CMD, int Motor_in1, int Motor_in2, int Motor_en, int Pulse, int Angle_Pulse, int Angle, int step_angle, int angle_rst, int Angle_Hold, int Motor_Status){
    if(digitalRead(angle_rst)){
      Angle=0;
      holdingRegs[Angle_Hold]=Angle;       
    }
     if(holdingRegs[Motor_CMD]>0){
        digitalWrite(Motor_in1, LOW); 
        digitalWrite(Motor_in2, HIGH);
        digitalWrite(Motor_en, HIGH);
        if (Pulse!=digitalRead(Angle_Pulse)){
          Angle=Angle+step_angle;
          Pulse=digitalRead(Angle_Pulse);
          }
    holdingRegs[Motor_Status]=1;
    }
    else if(holdingRegs[Motor_CMD]<0){
        digitalWrite(Motor_in1, HIGH); 
        digitalWrite(Motor_in2, LOW);
        digitalWrite(Motor_en, HIGH);
        if (Pulse!=digitalRead(Angle_Pulse)){
          Angle=Angle-step_angle;
          Pulse=digitalRead(Angle_Pulse);
        } 
    holdingRegs[Motor_Status]=1;  
    } 
    else{
        digitalWrite(Motor_en, LOW);  
        holdingRegs[Motor_Status]=0;
    }
    return Angle,Pulse;

}



int DC_Motor(int Motor_Pos_CMD, int Angle, int Motor_in1, int Motor_in2, int Motor_en, int Angle_Pulse, int Pulse, int Angle_Hold, int Motor_Status, int step_angle, int angle_rst){
  if(digitalRead(angle_rst)){
    Angle=0;
    holdingRegs[Angle_Hold]=Angle;       
    }
  if(Angle!=holdingRegs[Motor_Pos_CMD]){  // Motor Command Procedure
    if (Angle>holdingRegs[Motor_Pos_CMD]){
        digitalWrite(Motor_in1, LOW); 
        digitalWrite(Motor_in2, HIGH);
        digitalWrite(Motor_en, HIGH);
        if (Pulse!=digitalRead(Angle_Pulse)){
          Angle=Angle+step_angle;
          Pulse=digitalRead(Angle_Pulse);
          }
        holdingRegs[Angle_Hold]=Angle;       
      }
    else if (Angle<holdingRegs[Motor_Pos_CMD]){
        digitalWrite(Motor_in1, HIGH); 
        digitalWrite(Motor_in2, LOW);
        digitalWrite(Motor_en, HIGH);
        if (Pulse!=digitalRead(Angle_Pulse)){
          Angle=Angle-step_angle;
          Pulse=digitalRead(Angle_Pulse);
          }
        holdingRegs[Angle_Hold]=Angle;       
      } 
    holdingRegs[Motor_Status]=1;
  }
  else{
    digitalWrite(Motor_en, LOW);
    holdingRegs[Motor_Status]=0;
  }
    return Angle,Pulse;
}


void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
  
  holdingRegs[Main_Motor_CMD]=1;
  holdingRegs[Motor2_CMD]=1;
  
  
  if(holdingRegs[Man_Auto]) {
    
    (Main_Angle,Main_Pulse)=DC_Motor_Manual(Main_Motor_CMD, Main_Motor_in1, Main_Motor_in2, Main_Motor_en, Main_Angle_Pulse, Main_Pulse, Main_Angle, step_angle, Main_Angle_rst, Angle1, Main_Motor_Status);
    (Motor_2_Angle,Angle_2_Pulse)=DC_Motor_Manual(Motor2_CMD, Motor2_in1, Motor2_in2, Motor2_en, Angle2_Pulse, Angle_2_Pulse, Motor_2_Angle, step_angle, Angle2_Rst, Angle2, Motor2_Status);
    (Motor_2_Angle,Angle_2_Pulse)=DC_Motor_Manual(Motor3_CMD, Motor3_in1, Motor3_in2, Motor3_en, Angle3_Pulse, Angle_3_Pulse, Motor_3_Angle, step_angle, Angle3_Rst, Angle3, Motor3_Status);

    if(digitalRead(StepMo_Rst)){
      Step_Pos=0;
      holdingRegs[StepperMotor_Pos]=0;      
      if (holdingRegs[StepperMotor_CMD]>0){  // Stepper Motor Command Procedure
        holdingRegs[StepperMotor_Status]=1;
        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
        stepper.step(5);
        Step_Pos=Step_Pos+5;
        holdingRegs[StepperMotor_Pos]=Step_Pos; 
        holdingRegs[StepperMotor_Status]=0;     
      }
    }
    else{
      if (holdingRegs[StepperMotor_CMD]<0){  // Stepper Motor Command Procedure
        holdingRegs[StepperMotor_Status]=1;
        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
        stepper.step(-5);
        Step_Pos=Step_Pos-5;
        holdingRegs[StepperMotor_Pos]=Step_Pos; 
        holdingRegs[StepperMotor_Status]=0;     
      }
    }
  }
  else{
    if(digitalRead(StepMo_Rst)){
      Step_Pos=0;
      holdingRegs[StepperMotor_Pos]=0;      
    }
  
    if (Step_Pos != holdingRegs[StepperMotor_Pos_CMD]){  // Stepper Motor Command Procedure
      Steps=(holdingRegs[StepperMotor_Pos_CMD]-Step_Pos);
      if(Steps>0){
        holdingRegs[StepperMotor_Status]=1;
        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
        stepper.step(Steps);
        Step_Pos=Step_Pos+Steps;
        holdingRegs[StepperMotor_Pos]=Step_Pos; 
        holdingRegs[StepperMotor_Status]=0;     
      }
    }
    (Main_Angle,Main_Pulse)=DC_Motor(Main_Motor_Pos_CMD, Main_Angle, Main_Motor_in1, Main_Motor_in2, Main_Motor_en, Main_Pulse, Main_Angle_Pulse, Angle1, Main_Motor_Status, step_angle, Main_Angle_rst);
    (Motor_2_Angle,Angle_2_Pulse)=DC_Motor(Motor2_Pos_CMD, Motor_2_Angle, Motor2_in1, Motor2_in2, Motor2_en, Angle_2_Pulse, Angle2_Pulse, Angle2, Motor2_Status, step_angle, Angle2_Rst);
    (Motor_3_Angle,Angle_3_Pulse)=DC_Motor(Motor3_Pos_CMD, Motor_3_Angle, Motor3_in1, Motor3_in2, Motor3_en, Angle_3_Pulse, Angle3_Pulse, Angle3, Motor3_Status, step_angle, Angle3_Rst);   
  }
  
     
  }
  

