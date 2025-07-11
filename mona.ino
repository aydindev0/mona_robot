#include <Wire.h>
#include "Mona_ESP_lib.h"
#include <ESP32Encoder.h> 

ESP32Encoder right_encoder;
ESP32Encoder left_encoder;

int right_new_pos, right_old_pos, left_new_pos, left_old_pos;
float right_vel, left_vel, right_ref_vel, left_ref_vel, pulses_in_time;
int dead_zone;
// set sample time
float Ts = 0.01;

float start_time, current_time, elapsed_time;

float delta_right, delta_left, rotation_angle_right, rotation_angle_left;
int dead_zone_right = 30;
int dead_zone_left = 29;
float control, err;


// Wheel radius is 15mm, every revolute is 3500 pulse
const float robot_radius = 40;
const float wheel_radius = 15;

// The post-gearbox resolution is 3500 pulse per wheel revolution and 
// the radius of wheel is 15mm. In this case, the following scale_encoder
// value should be set so that the measurement results show the linear displacement
// in mm at the Serial Monitor/Plotter

// Q=E/m
// M=E/Q
// M=2pi*15mm/3500
// M=0.027

const float scale_encoder = 0.027;    
const float pi = 3.14159; 





void setup(){
    Mona_ESP_init();
    Serial.begin(115200);
    right_encoder.attachHalfQuad ( Mot_right_feedback, Mot_right_feedback_2 );
    left_encoder.attachHalfQuad( Mot_left_feedback_2, Mot_left_feedback );

	// clear starting values
	right_encoder.clearCount();
	left_encoder.clearCount();

	// set the lastToggle
	current_time = millis();
    start_time = current_time;

    // set the initial positions

    right_old_pos = 0;
    left_old_pos = 0;
    delay(5000);
    // Left_mot_forward(128);
    // Right_mot_forward(128);
        
    // 100 percent duty cycle    
    //Left_mot_forward(255);
    //Right_mot_forward(255);

    }



void position(){
  elapsed_time = (millis()-current_time)/1000;    // elapsed time in seconds
  right_new_pos = right_encoder.getCount();
  left_new_pos = left_encoder.getCount();
  // angular displacement = p/ppr * 2pi
  delta_right = right_new_pos - right_old_pos;
  delta_left = left_new_pos - left_old_pos; 
  // rotational angle = delta s / r
  rotation_angle_right = (delta_right * scale_encoder) / wheel_radius;
  rotation_angle_left = (delta_left * scale_encoder) / wheel_radius;
  // w = rotational angle / dt
  right_vel = rotation_angle_right / elapsed_time;
  left_vel = rotation_angle_left / elapsed_time;
  Serial.print("Right velocity: ");
  Serial.print(right_vel, 2);
  Serial.print("Left velocity: ");
  Serial.print(left_vel, 2);
  Serial.println();
  right_old_pos = right_new_pos; 
  left_old_pos = left_new_pos;
}


void Propotional(float ref, float act, float Kp){
  err = (ref-act);
  control = Kp*err;
}


void loop(){
  // below code gets dead zone
  // while (current_time-start_time<10000){
  //   position();
  //   if (right_vel<=0){
  //   dead_zone = dead_zone + 1;
  //   Right_mot_forward(dead_zone);
  // }
  // else{
  //   Motors_stop();
  //   Serial.end();
  //   delay(3000);
  // }
  // Serial.print("The dead zone is: ");
  // Serial.print(dead_zone);
  // Serial.println();
  //   Serial.print(scale_encoder*right_old_pos);
  //   Serial.print("\t");
  //   Serial.print(scale_encoder*left_old_pos);
  //   Serial.println();

  // correct angular velocity using the dead zone
  while(true){
    position();
    Propotional(pi, right_vel, 100.0);
    Right_mot_forward(control+dead_zone_right);
    Propotional(pi, left_vel, 100.0);
    Left_mot_forward(control+dead_zone_left);
    Serial.print("Right vel: ");
    Serial.print(right_vel);
    Serial.println();
    Serial.print("Left vel: ");
    Serial.print(left_vel);
    Serial.println();
    current_time = millis();
    delay(Ts*1000);
    }
}