#include <Arduino.h>
#include <Servo.h>

#include "Quadruped.h"

Quadruped::Quadruped(float _w, float _l, float _l1, float _l2, float _l3){
  W = _w;
  L = _l;
  L1 = _l1;
  L2 = _l2;
  L3 = _l3;
  
  init_local_frames();
  update_frames_wrt_cog();
}

void Quadruped::init_local_frames(){
  local_frames.rf.x = 14.899;
  local_frames.rf.y = -60;
  local_frames.rf.z = 0;
  local_frames.lf.x = -14.899;
  local_frames.lf.y = -60;
  local_frames.lf.z = 0;
  local_frames.rb.x = 14.899;
  local_frames.rb.y = -60;
  local_frames.rb.z = 0;
  local_frames.lb.x = -14.899;
  local_frames.lb.y = -60;
  local_frames.lb.z = 0;
}

void Quadruped::attach_servos(){
  rfh1.attach(servo_pins[0]);
  rfh2.attach(servo_pins[1]);
  rfk.attach(servo_pins[2]);
  lfh1.attach(servo_pins[3]);
  lfh2.attach(servo_pins[4]);
  lfk.attach(servo_pins[5]);
  rbh1.attach(servo_pins[6]);
  rbh2.attach(servo_pins[7]);
  rbk.attach(servo_pins[8]);
  lbh1.attach(servo_pins[9]);
  lbh2.attach(servo_pins[10]);
  lbk.attach(servo_pins[11]);
}

void Quadruped::init_initialise(){
  attach_servos();

  rfh1.write(zero_positions[0]);
  rfh2.write(zero_positions[1]);
  rfk.write(zero_positions[2]);
  lfh1.write(zero_positions[3]);
  lfh2.write(zero_positions[4]);
  lfk.write(zero_positions[5]);
  rbh1.write(zero_positions[6]);
  rbh2.write(zero_positions[7]);
  rbk.write(zero_positions[8]);
  lbh1.write(zero_positions[9]);
  lbh2.write(zero_positions[10]);
  lbk.write(zero_positions[11]);
  
  delay(1000);
}

void Quadruped::initialise(){
  IK();
  attach_servos();
  delay(1000);
}

void Quadruped::IK(){
  float theta1rf = -atan2(-local_frames.rf.y, local_frames.rf.x)-atan2(sqrt(sq(local_frames.rf.x)+sq(local_frames.rf.y)-sq(L1)), -L1);
  float Drf = (sq(local_frames.rf.x)+sq(local_frames.rf.y)-sq(L1)+sq(-local_frames.rf.z)-sq(L2)-sq(L3))/(2*L2*L3);
  float theta3rf = atan2(-sqrt(1-sq(Drf)), Drf);
  float theta2rf = atan2(-local_frames.rf.z, sqrt(sq(local_frames.rf.x)+sq(local_frames.rf.y)-sq(L1)))-atan2(L3 * sin(theta3rf), L2+L3*cos(theta3rf));
  float theta1rf_deg = (180.0/M_PI)*theta1rf;
  float theta2rf_deg = (180.0/M_PI)*theta2rf;
  float theta3rf_deg = (180.0/M_PI)*theta3rf;

  float theta1lf = -atan2(-local_frames.lf.y, -local_frames.lf.x)-atan2(sqrt(sq(-local_frames.lf.x)+sq(local_frames.lf.y)-sq(L1)), -L1);
  float Dlf = (sq(-local_frames.lf.x)+sq(local_frames.lf.y)-sq(L1)+sq(-local_frames.lf.z)-sq(L2)-sq(L3))/(2*L2*L3);
  float theta3lf = atan2(-sqrt(1-sq(Dlf)), Dlf);
  float theta2lf = atan2(-local_frames.lf.z, sqrt(sq(-local_frames.lf.x)+sq(local_frames.lf.y)-sq(L1)))-atan2(L3 * sin(theta3lf), L2+L3*cos(theta3lf));
  float theta1lf_deg = (180.0/M_PI)*theta1lf;
  float theta2lf_deg = (180.0/M_PI)*theta2lf;
  float theta3lf_deg = (180.0/M_PI)*theta3lf;

  float theta1rb = -atan2(-local_frames.rb.y, local_frames.rb.x)-atan2(sqrt(sq(local_frames.rb.x)+sq(local_frames.rb.y)-sq(L1)), -L1);
  float Drb = (sq(local_frames.rb.x)+sq(local_frames.rb.y)-sq(L1)+sq(local_frames.rb.z)-sq(L2)-sq(L3))/(2*L2*L3);
  float theta3rb = atan2(-sqrt(1-sq(Drb)), Drb);
  float theta2rb = atan2(local_frames.rb.z, sqrt(sq(local_frames.rb.x)+sq(local_frames.rb.y)-sq(L1)))-atan2(L3 * sin(theta3rb), L2+L3*cos(theta3rb));
  float theta1rb_deg = (180.0/M_PI)*theta1rb;
  float theta2rb_deg = (180.0/M_PI)*theta2rb;
  float theta3rb_deg = (180.0/M_PI)*theta3rb;

  float theta1lb = -atan2(-local_frames.lb.y, -local_frames.lb.x)-atan2(sqrt(sq(-local_frames.lb.x)+sq(local_frames.lb.y)-sq(L1)), -L1);
  float Dlb = (sq(-local_frames.lb.x)+sq(local_frames.lb.y)-sq(L1)+sq(local_frames.lb.z)-sq(L2)-sq(L3))/(2*L2*L3);
  float theta3lb = atan2(-sqrt(1-sq(Dlb)), Dlb);
  float theta2lb = atan2(local_frames.lb.z, sqrt(sq(-local_frames.lb.x)+sq(local_frames.lb.y)-sq(L1)))-atan2(L3 * sin(theta3lb), L2+L3*cos(theta3lb));
  float theta1lb_deg = (180.0/M_PI)*theta1lb;
  float theta2lb_deg = (180.0/M_PI)*theta2lb;
  float theta3lb_deg = (180.0/M_PI)*theta3lb;
  
  rfh1.write(zero_positions[0] + (-180 - theta1rf_deg));
  rfh2.write(zero_positions[1] - theta2rf_deg);
  rfk.write(zero_positions[2] + theta3rf_deg);
  
  lfh1.write(zero_positions[3] - (-180 - theta1lf_deg));
  lfh2.write(zero_positions[4] + theta2lf_deg);
  lfk.write(zero_positions[5] - theta3lf_deg);

  rbh1.write(zero_positions[6] - (-180 - theta1rb_deg));
  rbh2.write(zero_positions[7] + theta2rb_deg);
  rbk.write(zero_positions[8] - theta3rb_deg);
  
  lbh1.write(zero_positions[9] + (-180 - theta1lb_deg));
  lbh2.write(zero_positions[10] - theta2lb_deg);
  lbk.write(zero_positions[11] + theta3lb_deg);

//  Serial.println("finshed moving");
}

void Quadruped::IK_one_leg(String leg, float x, float y, float z){
  if (leg == "rf"){
    local_frames.rf.x += x;
    local_frames.rf.y += y;
    local_frames.rf.z += z;
  }
  else if (leg == "lf"){
    local_frames.lf.x += x;
    local_frames.lf.y += y;
    local_frames.lf.z += z;
  }
  else if (leg == "rb"){
    local_frames.rb.x += x;
    local_frames.rb.y += y;
    local_frames.rb.z += z;
  }
  else if (leg == "lb"){
    local_frames.lb.x += x;
    local_frames.lb.y += y;
    local_frames.lb.z += z;
  }
  IK();
}

void Quadruped::update_frames_wrt_cog(){
  frames_wrt_cog.rf.x = local_frames.rf.x + W/2;
  frames_wrt_cog.rf.y = local_frames.rf.y;
  frames_wrt_cog.rf.z = local_frames.rf.z + L/2;

  frames_wrt_cog.lf.x = local_frames.lf.x - W/2;
  frames_wrt_cog.lf.y = local_frames.lf.y;
  frames_wrt_cog.lf.z = local_frames.lf.z + L/2;

  frames_wrt_cog.rb.x = local_frames.rb.x + W/2;
  frames_wrt_cog.rb.y = local_frames.rb.y;
  frames_wrt_cog.rb.z = local_frames.rb.z - L/2;

  frames_wrt_cog.lb.x = local_frames.lb.x - W/2;
  frames_wrt_cog.lb.y = local_frames.lb.y;
  frames_wrt_cog.lb.z = local_frames.lb.z - L/2;
}

void Quadruped::update_local_frames(){
  local_frames.rf.x = frames_wrt_cog.rf.x - W/2;
  local_frames.rf.y = frames_wrt_cog.rf.y;
  local_frames.rf.z = frames_wrt_cog.rf.z - L/2;

  local_frames.lf.x = frames_wrt_cog.lf.x + W/2;
  local_frames.lf.y = frames_wrt_cog.lf.y;
  local_frames.lf.z = frames_wrt_cog.lf.z - L/2;

  local_frames.rb.x = frames_wrt_cog.rb.x - W/2;
  local_frames.rb.y = frames_wrt_cog.rb.y;
  local_frames.rb.z = frames_wrt_cog.rb.z + L/2;

  local_frames.lb.x = frames_wrt_cog.lb.x + W/2;
  local_frames.lb.y = frames_wrt_cog.lb.y;
  local_frames.lb.z = frames_wrt_cog.lb.z + L/2;
}

void Quadruped::translate_cog(float x, float z){
  cog_x += x;
  cog_z += z;

  Serial.print(cog_x);
  Serial.print(", ");
  Serial.println(cog_z);
  
  local_frames.rf.x -= x;
  local_frames.lf.x -= x;
  local_frames.rb.x -= x;
  local_frames.lb.x -= x;

  local_frames.rf.z -= z;
  local_frames.lf.z -= z;
  local_frames.rb.z -= z;
  local_frames.lb.z -= z;

  IK();
}

void Quadruped::translate_cog_on_support_polygon(String swing_leg){
  float support_polygon_centroid_x;
  float support_polygon_centroid_z;
  
  update_frames_wrt_cog();

//  Serial.print("(");
//  Serial.print(frames_wrt_cog.rf.x);
//  Serial.print(", ");
//  Serial.print(frames_wrt_cog.rf.z);
//  Serial.println(")");
//
//  Serial.print("(");
//  Serial.print(frames_wrt_cog.lf.x);
//  Serial.print(", ");
//  Serial.print(frames_wrt_cog.lf.z);
//  Serial.println(")");
//
//  Serial.print("(");
//  Serial.print(frames_wrt_cog.rb.x);
//  Serial.print(", ");
//  Serial.print(frames_wrt_cog.rb.z);
//  Serial.println(")");
//
//  Serial.print("(");
//  Serial.print(frames_wrt_cog.lb.x);
//  Serial.print(", ");
//  Serial.print(frames_wrt_cog.lb.z);
//  Serial.println(")");
  
  
  if (swing_leg == "rf"){
    support_polygon_centroid_x = (frames_wrt_cog.lf.x + frames_wrt_cog.rb.x + frames_wrt_cog.lb.x) / 3.0;
    support_polygon_centroid_z = (frames_wrt_cog.lf.z + frames_wrt_cog.rb.z + frames_wrt_cog.lb.z) / 3.0;
  }
  else if (swing_leg == "lf"){
    support_polygon_centroid_x = (frames_wrt_cog.rf.x + frames_wrt_cog.rb.x + frames_wrt_cog.lb.x) / 3.0;
    support_polygon_centroid_z = (frames_wrt_cog.rf.z + frames_wrt_cog.rb.z + frames_wrt_cog.lb.z) / 3.0;
  }
  else if (swing_leg == "rb"){
    support_polygon_centroid_x = (frames_wrt_cog.rf.x + frames_wrt_cog.lf.x + frames_wrt_cog.lb.x) / 3.0;
    support_polygon_centroid_z = (frames_wrt_cog.rf.z + frames_wrt_cog.lf.z + frames_wrt_cog.lb.z) / 3.0;
  }
  else if (swing_leg == "lb"){
    support_polygon_centroid_x = (frames_wrt_cog.rf.x + frames_wrt_cog.lf.x + frames_wrt_cog.rb.x) / 3.0;
    support_polygon_centroid_z = (frames_wrt_cog.rf.z + frames_wrt_cog.lf.z + frames_wrt_cog.rb.z) / 3.0;
  }

//  Serial.println("Support polygon centroid");
//  Serial.print("(");
//  Serial.print(support_polygon_centroid_x);
//  Serial.print(", ");
//  Serial.print(support_polygon_centroid_z);
//  Serial.println(")");
  
  float old_x = 0;
  float old_z = 0;
  for (int i = 0; i<=100; i++){
    float new_x = i * support_polygon_centroid_x / 100.0;
    float new_z = i * support_polygon_centroid_z / 100.0;
    translate_cog(new_x - old_x, new_z - old_z);
    old_x = new_x;
    old_z = new_z;
    delay(1);
  }
  
}
