#include "Quadruped.h"

Quadruped robot(48.302, 91, 15.151, 40, 40);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  robot.init_initialise();
  robot.initialise();
  
  delay(100);
  robot.translate_cog_on_support_polygon("rf");
//  delay(1000);
  robot.IK_one_leg("rf", 0,20,10);
  delay(100);
  robot.IK_one_leg("rf", 0,-20,5);

  delay(100);
  robot.translate_cog_on_support_polygon("lb");
//  delay(1000);
  robot.IK_one_leg("lb", 0,20,10);
  delay(100);
  robot.IK_one_leg("lb", 0,-20,5);

  delay(100);
  robot.translate_cog_on_support_polygon("lf");
//  delay(1000);
  robot.IK_one_leg("lf", 0,20,20);
  delay(100);
  robot.IK_one_leg("lf", 0,-20,10);

  delay(100);
  robot.translate_cog_on_support_polygon("rb");
//  delay(1000);
  robot.IK_one_leg("rb", 0,20,20);
  delay(100);
  robot.IK_one_leg("rb", 0,-20,10);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  robot.translate_cog_on_support_polygon("rf");
//  delay(1000);
  robot.IK_one_leg("rf", 0,20,20);
  delay(100);
  robot.IK_one_leg("rf", 0,-20,10);

  delay(100);
  robot.translate_cog_on_support_polygon("lb");
//  delay(1000);
  robot.IK_one_leg("lb", 0,20,20);
  delay(100);
  robot.IK_one_leg("lb", 0,-20,10);

  delay(100);
  robot.translate_cog_on_support_polygon("lf");
//  delay(1000);
  robot.IK_one_leg("lf", 0,20,20);
  delay(100);
  robot.IK_one_leg("lf", 0,-20,10);

  delay(100);
  robot.translate_cog_on_support_polygon("rb");
//  delay(1000);
  robot.IK_one_leg("rb", 0,20,20);
  delay(100);
  robot.IK_one_leg("rb", 0,-20,10);
}
