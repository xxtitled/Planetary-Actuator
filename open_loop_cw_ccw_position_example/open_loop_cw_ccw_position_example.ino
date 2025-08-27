#include <SimpleFOC.h>

// 1) BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// 2) Control variables
float target_angle     = 0;  // rad (+CW, -CCW)
float movement_velocity = 5;  // rad/s

// 3) Commander (serial commands)
Commander command = Commander(Serial);
void doTarget(char* cmd)  { command.scalar(&target_angle, cmd); }
void doLimit(char* cmd)   { command.scalar(&motor.voltage_limit, cmd); }
void doVelocity(char* cmd){ command.scalar(&movement_velocity, cmd); }

void setup() {
  // serial + debug
  Serial.begin(115200);
  while(!Serial);
  SimpleFOCDebug::enable(&Serial);

  // driver setup
  driver.voltage_power_supply = 12;
  driver.voltage_limit        = 6;
  if(!driver.init()) {
    Serial.println("Driver init failed!");
    while(1);
  }
  motor.linkDriver(&driver);

  // motor settings
  motor.voltage_limit  = 3;
  motor.velocity_limit = movement_velocity;
  motor.controller     = MotionControlType::angle_openloop;
  if(!motor.init()) {
    Serial.println("Motor init failed!");
    while(1);
  }

  // register commands
  command.add('T', doTarget,   "target angle [rad], +CW / -CCW");
  command.add('L', doLimit,    "voltage limit [V]");
  command.add('V', doVelocity, "movement velocity [rad/s]");

  Serial.println("Motor ready!");
  Serial.println("Commands:");
  Serial.println("  T<angle> : set target angle (+CW, -CCW)");
  Serial.println("  V<vel>   : set movement velocity (rad/s)");
  Serial.println("  L<volt>  : set voltage limit (V)");
}

void loop() {
  // update velocity limit each loop
  motor.velocity_limit = movement_velocity;
  // move open-loop to target angle
  motor.move(target_angle);
  // parse serial commands
  command.run();
}