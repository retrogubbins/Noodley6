// Pete SumoBot 2017
//
#include <MsTimer2.h>

#include "ESC.h"
#include <SharpIR.h>

#define ir A0
#define model 1080
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

#define R_DISTANCE A2 
#define L_DISTANCE A3 
#define THREATZONE_LIMIT 30
#define PUSHZONE_LIMIT 15
#define FULL_CONTACT 0
#define NO_CONTACT 1
#define LEFT_CONTACT 2
#define RIGHT_CONTACT 3

#define BUTTON_RED 3
#define BUTTON_GND 4

#define R_LINE_SENSOR A6 
#define L_LINE_SENSOR A7 

#define L_LINE_SENSOR_THRESH 50
#define R_LINE_SENSOR_THRESH L_LINE_SENSOR_THRESH

#define R_MOTOR 5
#define L_MOTOR 6

#define R_MOTORS_MINSPEED 50
#define R_MOTORS_MAXSPEED 200

#define L_MOTORS_MINSPEED 60
#define L_MOTORS_MAXSPEED 200

#define R_ESC_MIDVAL  1400
#define R_REV_SPEED_MIN (R_ESC_MIDVAL - R_MOTORS_MINSPEED)
#define R_REV_SPEED_MAX (R_ESC_MIDVAL - R_MOTORS_MAXSPEED)
#define R_FWD_SPEED_MIN (R_ESC_MIDVAL + R_MOTORS_MINSPEED)
#define R_FWD_SPEED_MAX (R_ESC_MIDVAL + R_MOTORS_MAXSPEED)

#define L_ESC_MIDVAL R_ESC_MIDVAL 
#define L_REV_SPEED_MIN (L_ESC_MIDVAL - L_MOTORS_MINSPEED)
#define L_REV_SPEED_MAX (L_ESC_MIDVAL - L_MOTORS_MAXSPEED)
#define L_FWD_SPEED_MIN (L_ESC_MIDVAL + L_MOTORS_MINSPEED)  // +60
#define L_FWD_SPEED_MAX (L_ESC_MIDVAL + L_MOTORS_MAXSPEED)


ESC R_ESC (R_MOTOR, R_REV_SPEED_MAX, R_FWD_SPEED_MAX, 500);      //500           // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC L_ESC (L_MOTOR, L_REV_SPEED_MAX, L_FWD_SPEED_MAX, 500);  

SharpIR L_DIST(L_DISTANCE, model);
SharpIR R_DIST(R_DISTANCE, model);

unsigned int L_accel_rate = 4;
unsigned int R_accel_rate = 2;

volatile unsigned int L_desired_speed = L_ESC_MIDVAL;
volatile unsigned int R_desired_speed = R_ESC_MIDVAL;
volatile unsigned int L_current_speed = L_ESC_MIDVAL;
volatile unsigned int R_current_speed = R_ESC_MIDVAL;

unsigned int L_FWD_SPEED_RANGE,R_FWD_SPEED_RANGE,L_REV_SPEED_RANGE,R_REV_SPEED_RANGE;

#define IDLE_STATE 0
#define DROP_FLAP_STATE 1
#define SEARCH_LEFT_STATE 2
#define SEARCH_RIGHT_STATE 3
#define AVOID_LEFT_LINE_STATE 4
#define AVOID_RIGHT_LINE_STATE 5
#define FORWARD_STATE 6
#define DESTROY_STATE 7
#define SEARCH_STATE 8


unsigned int current_state = IDLE_STATE;



int data = L_ESC_MIDVAL;
volatile unsigned long blinkCount = 0; // use volatile for shared variables
unsigned long blinkCopy;  // holds a copy of the blinkCount

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting noodly");  
  delay(1000);
  // ARM ESC controllers
  R_ESC.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  L_ESC.arm();                                            // Send the Arm value so the ESC will be ready to take commands

//L_ESC.calib();  
//R_ESC.calib();  

  L_FWD_SPEED_RANGE = L_FWD_SPEED_MAX - L_FWD_SPEED_MIN;
  R_FWD_SPEED_RANGE = R_FWD_SPEED_MAX - R_FWD_SPEED_MIN;

  L_REV_SPEED_RANGE = L_REV_SPEED_MIN - L_REV_SPEED_MAX;
  R_REV_SPEED_RANGE = R_REV_SPEED_MIN - R_REV_SPEED_MAX;

  Serial.print("L_FWD_SPEED_RANGE ");
  Serial.print(L_FWD_SPEED_RANGE);
  Serial.print("  R_FWD_SPEED_RANGE ");
  Serial.print(R_FWD_SPEED_RANGE);
  Serial.print("  L_REV_SPEED_RANGE ");
  Serial.print(L_REV_SPEED_RANGE);
  Serial.print("  R_REV_SPEED_RANGE ");
  Serial.println(R_REV_SPEED_RANGE);
  current_state = IDLE_STATE;
delay(1000);
motors_fast_stop();
delay(1000);
  // initialize timer2 
  MsTimer2::set(20, timer2_isr); // 500ms period
  MsTimer2::start();

//L_forward(10);
//delay(2000);
////L_forward(50);
////delay(2000);
////
//motors_slow_stop();
////delay(2000);
////
//L_reverse(10);
//delay(2000);
////
////L_reverse(50);
////delay(2000);
////forward(20);
//motors_slow_stop();

pinMode(BUTTON_RED,INPUT_PULLUP);
pinMode(BUTTON_GND,OUTPUT);
digitalWrite(BUTTON_GND,LOW);

test_for_start_button();
delay(5000);
      
//  spot_turn_right(10,20);
// delay(2000);   
//  spot_turn_left(10,20);


  
}

void loop()
{
  if(L_line_det())
  {
    current_state = AVOID_LEFT_LINE_STATE;
  }
  if(R_line_det())
  {
    current_state = AVOID_RIGHT_LINE_STATE;
  }
  
  switch(current_state)
  {
    case IDLE_STATE:
      Serial.println("idle state");
      current_state = SEARCH_STATE;
      break;
    case AVOID_LEFT_LINE_STATE:
      Serial.println("motors fast stop");
      motors_fast_stop();
      Serial.println("avoid move");
      reverse(20);
      delay(700);
      motors_fast_stop();
      spot_turn_right(13,20);
      current_state = IDLE_STATE;
      break;
    case AVOID_RIGHT_LINE_STATE:
      Serial.println("motors fast stop");
      motors_fast_stop();
      Serial.println("avoid move");
      reverse(20);
      delay(700);
      motors_fast_stop();
      spot_turn_left(13,20);
      current_state = IDLE_STATE;
      break;

    case SEARCH_STATE:
      forward(2);
      if(distance_threat() != NO_CONTACT)
      { 
         current_state = DESTROY_STATE;
      }
      break;
      
    case DESTROY_STATE:
//      forward(10);
      unsigned char dest = destroy();
     
      if(dest == NO_CONTACT)
      { 
       //   motors_fast_stop();
          current_state = SEARCH_STATE;
      }
      break;
    
  }


 //print_debug();
}


void loop2()
{
 unsigned char dest = destroy();
      if(dest == NO_CONTACT)
      { 
        
      }

// print_debug();
}


unsigned char distance_threat()
{
  int  l_dist = L_DIST.distance();
  int  r_dist = R_DIST.distance();
  if(l_dist < THREATZONE_LIMIT && r_dist < THREATZONE_LIMIT)
  {
     return FULL_CONTACT;
  }
  else if(l_dist > THREATZONE_LIMIT && r_dist > THREATZONE_LIMIT)
  {
     return NO_CONTACT;
  }
  else if(l_dist <  r_dist)
  {
     return LEFT_CONTACT;
  }
  else  
     return RIGHT_CONTACT;
}
 
int  l_dist,r_dist;
int  l_perc,r_perc;
 
unsigned char destroy()
{
  l_dist = L_DIST.distance();
  r_dist = R_DIST.distance();
  if(l_dist > THREATZONE_LIMIT && r_dist > THREATZONE_LIMIT)
    return NO_CONTACT;

  if(l_dist > THREATZONE_LIMIT)
    l_dist = THREATZONE_LIMIT;
  if(r_dist > THREATZONE_LIMIT)
    r_dist = THREATZONE_LIMIT;
    
  
  
  int diff = l_dist - r_dist; 
//  if(abs(diff) < 5)
//  {
//    forward(20);
//    Serial.println("PUSHING");
//    return FULL_CONTACT;  
//  }
//  else 
  //if(l_dist < r_dist)
  {
        l_perc = (l_dist * 3 );
        r_perc = (r_dist * 3);
        R_forward(r_perc);
        L_forward(l_perc);
//        destroy_debug();
        return FULL_CONTACT;
  }
//  else if(l_dist > r_dist)
//  {
//        l_perc = (l_dist - r_dist) / 2;
//        r_perc = (r_dist - l_dist) / 2;
//        R_forward(r_perc);
//        L_forward(l_perc);
//        destroy_debug();
//        return FULL_CONTACT;
//  }
  return NO_CONTACT;
}

void destroy_debug()
{
  Serial.print("  LD:");
  Serial.print(l_dist);
  Serial.print("  RD:");
  Serial.print(r_dist);
  Serial.print("   LM(");
  Serial.print(L_desired_speed);
  Serial.print("): ");
  Serial.print(L_current_speed);
  Serial.print(" / ");
  Serial.print(l_perc);
  
  Serial.print("    RM(");
  Serial.print(R_desired_speed);
  Serial.print("): ");
  Serial.print(R_current_speed);  
  Serial.print(" / ");
  Serial.println(r_perc);
}


void motors_spin_left(unsigned int perc)
{
  L_reverse(perc);
  R_forward(perc);
}

void motors_spin_right(unsigned int perc)
{
  R_reverse(perc);
  L_forward(perc);
}

void L_forward(unsigned int perc)
{
  L_desired_speed = L_FWD_SPEED_MIN + (L_FWD_SPEED_RANGE*perc)/100;
}

void R_forward(unsigned int perc)
{
  R_desired_speed = R_FWD_SPEED_MIN + (R_FWD_SPEED_RANGE*perc)/100;
}

void L_reverse(unsigned int perc)
{  
  L_desired_speed = L_REV_SPEED_MIN - (L_REV_SPEED_RANGE*perc)/100;
}

void R_reverse(unsigned int perc)
{
  R_desired_speed = R_REV_SPEED_MIN - (R_REV_SPEED_RANGE*perc)/100;
}



void forward(unsigned int perc)
{
  L_forward(perc);
  R_forward(perc);
}

void reverse(unsigned int perc)
{
  L_reverse(perc);
  R_reverse(perc);
}

void spot_turn_right(unsigned int angle, unsigned int perc)
{
  L_forward(perc);
  R_reverse(perc);
  delay(angle * 50);
  motors_slow_stop();
  delay(angle * 50);
}

void spot_turn_left(unsigned int angle, unsigned int perc)
{
  R_forward(perc);
  L_reverse(perc);
  delay(angle * 50);
  motors_slow_stop();
  delay(angle * 50);
}

void print_debug()
{
  Serial.print("LL:");
  if(L_line_det()) Serial.print("DET");
  else  Serial.print("   ");
  Serial.print("  RL:");
  if(R_line_det()) Serial.print("DET");
  else  Serial.print("   ");
  Serial.print("  LD:");
  Serial.print(L_DIST.distance());
  Serial.print("  RD:");
  Serial.print(R_DIST.distance());
  Serial.print("   LM(");
  Serial.print(L_desired_speed);
  Serial.print("): ");
  Serial.print(L_current_speed);
  Serial.print("   RM(");
  Serial.print(R_desired_speed);
  Serial.print("): ");
  Serial.println(R_current_speed);
}
void loop3() 
{
 
  // put your main code here, to run repeatedly:
  print_debug();

//  noInterrupts();
//  blinkCopy = blinkCount;
//  interrupts();
//
//  Serial.print("   blinkCount = ");
//  Serial.println(blinkCopy);


  if (Serial.available() > 0) 
  {
      Serial.println("PARSING");
    data = Serial.parseInt(); // Parse an Integer from Serial
      Serial.println("p done");
    if(data == 0)
     {
        motors_fast_stop();  
     }
     else
     {
      Serial.println("Setting escs");
//      L_reverse(data);
  //    R_reverse(data);

     forward(data);

//      L_desired_speed = data; 
      Serial.println("DONE");
    }
   }
   
}




void motors_fast_stop()
{
  L_desired_speed = L_ESC_MIDVAL;
  R_desired_speed = R_ESC_MIDVAL;
  L_current_speed = L_ESC_MIDVAL;
  R_current_speed = R_ESC_MIDVAL;
  R_ESC.speed(R_current_speed);                                    // tell ESC to go to the oESC speed value
  L_ESC.speed(L_current_speed);                                    // tell ESC to go to the oESC speed value
}

void motors_slow_stop()
{
  L_desired_speed = L_ESC_MIDVAL;
  R_desired_speed = R_ESC_MIDVAL;
}

void timer2_isr()
{
    blinkCount = blinkCount + 1;  // increase when LED turns on
    
    // LEFT ACCELERATION
    if(L_desired_speed != L_current_speed)
    {
      if(L_desired_speed > L_current_speed)
      {
        L_current_speed += L_accel_rate;
        if(L_current_speed > L_desired_speed)
          L_current_speed = L_desired_speed;
      }
      else if(L_desired_speed < L_current_speed)
      {
        L_current_speed -= L_accel_rate;
        if(L_current_speed < L_desired_speed)
          L_current_speed = L_desired_speed;
      }
      L_ESC.speed(L_current_speed);
    }

    // RIGHT ACCELERATION
    if(R_desired_speed != R_current_speed)
    {
      if(R_desired_speed > R_current_speed)
      {
        R_current_speed += R_accel_rate;
        if(R_current_speed > R_desired_speed)
          R_current_speed = R_desired_speed;
      }
      else if(R_desired_speed < R_current_speed)
      {
        R_current_speed -= R_accel_rate;
        if(R_current_speed < R_desired_speed)
          R_current_speed = R_desired_speed;
      }
      R_ESC.speed(R_current_speed);
    }

}


bool L_line_det()
{
  return(analogRead(L_LINE_SENSOR) > L_LINE_SENSOR_THRESH);
}

bool R_line_det()
{
  return(analogRead(R_LINE_SENSOR) > R_LINE_SENSOR_THRESH);
}



void test_for_start_button()
{
  bool startnow = false;
  while(startnow == false)
  {
    if(digitalRead(BUTTON_RED) == LOW)
    {
      delay(100);
      if(digitalRead(BUTTON_RED) == LOW)
      {
        startnow = true;
      }
    }
  }
}


