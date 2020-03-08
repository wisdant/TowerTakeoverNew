#include "main.h"
using namespace pros;


int autonNumber = 0;
bool redAlliance = true;
int start_ts;

// Define the main controller of the robot.
Controller master(E_CONTROLLER_MASTER);

// Define the robot's motors with their respective ports.
// Passing true for the second parameter reverses the motor.
Motor left_drive (LEFT_FRONT_PORT, false);

Motor left_rear_drive (LEFT_REAR_PORT, false);

Motor right_drive (RIGHT_FRONT_PORT, true);
Motor right_rear_drive (RIGHT_REAR_PORT, true);

Motor left_intake (LEFT_INTAKE_PORT, E_MOTOR_GEARSET_36, false);
Motor right_intake (RIGHT_INTAKE_PORT, E_MOTOR_GEARSET_36, true);

Motor lift (LIFT_PORT, E_MOTOR_GEARSET_36, true);
Motor hinge (HINGE_PORT, E_MOTOR_GEARSET_36, true);

Vision vision_sensor (VISION_PORT);

Imu imu_sensor (IMU_PORT);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User - leap day!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	int retNo = imu_sensor.reset();
	lcd::print(3, "Reset return code =%d", retNo);
	lcd::print(4, "PROS_ERR =%d", PROS_ERR);
	lcd::print(5, "errno =%d", errno);

  int iter = 0;
  while (imu_sensor.is_calibrating()) {
		lcd::print(2, "Calibrating...iter=%d", iter);
    iter += 10;
    delay(10);
  }

	lcd::print(2, "Calibration complete. It took %d ms", iter);
}

void a_move_drive(int right, int left) {
  left_drive.move(left);
  left_rear_drive.move(left);
  right_drive.move(right);
  right_rear_drive.move(right);
}

void a_move_intake(int power) {
  left_intake.move(power);
  right_intake.move(power);
}

void a_move_lift(int power) {
  lift.move(power);
}

void a_move_hinge(int power) {
  hinge.move(power);
}

void a_set_drive_encoding(motor_encoder_units_e_t units) {
  left_drive.set_encoder_units(units);
  left_rear_drive.set_encoder_units(units);
  right_drive.set_encoder_units(units);
  right_rear_drive.set_encoder_units(units);
}

void a_tare_position(){
  left_drive.tare_position();
  left_rear_drive.tare_position();
  right_drive.tare_position();
  right_rear_drive.tare_position();
}

void move_by_distance(bool isAbsoluteMove, double FR_target, double RR_target,
  double FL_target, double RL_target,std::int32_t velocity, int timeout){
  // Converting the distance (inches) to ticks
  FR_target *= TICK_PER_INCH;
  RR_target *= TICK_PER_INCH;
  FL_target *= TICK_PER_INCH;
  RL_target *= TICK_PER_INCH;

  if (VERBOSE) {
    printf("Start moving. isAbsolute: %s, Timeout: %d, LF: %f, LR: %f, RF: %f, RR: %f, LF_target: %f, LR_target: %f, RF_target: %f, RR_target: %f  \n",
      isAbsoluteMove ? "Absolute" : "Relative", timeout,
      left_drive.get_position(), left_rear_drive.get_position(),
      right_drive.get_position(), right_rear_drive.get_position(),
      FL_target, RL_target, FR_target, RR_target
    );
  }

  if (isAbsoluteMove) {
    left_drive.move_absolute(FL_target, velocity);
    left_rear_drive.move_absolute(RL_target, velocity);
    right_drive.move_absolute(FR_target, velocity);
    right_rear_drive.move_absolute(RR_target, velocity);
  }
  else {
    left_drive.move_relative(FL_target, velocity);
    left_rear_drive.move_relative(RL_target, velocity);
    right_drive.move_relative(FR_target, velocity);
    right_rear_drive.move_relative(RR_target, velocity);
  }

  int counter = 0;    // Used to decide if timeout has reached.
  while (true) {
    counter++;
    if (VERBOSE) {
      printf("Counter: %d, FR: %f, RR: %f, FL: %f, RL: %f\n", counter,
        right_drive.get_position(), right_rear_drive.get_position(),
        left_drive.get_position(), left_rear_drive.get_position());
    }

    double LF_pos = left_drive.get_position();
    double LR_pos = left_rear_drive.get_position();
    double RF_pos = right_drive.get_position();
    double RR_pos = right_rear_drive.get_position();

    if ((abs(int(LF_pos - FL_target)) > abs(int(FL_target * 5/100)))
        || (abs(int(LR_pos - RL_target)) > abs(int(RL_target * 5/100)))
        || (abs(int(RF_pos - FR_target)) > abs(int(FR_target * 5/100)))
        || (abs(int(RR_pos - RR_target)) > abs(int(RR_target * 5/100)))
    ) {
      delay(20);
      if ((timeout!=-1) && (counter * 20 > timeout)) break;
    }
    else {
      delay(20);
      break;
    }
  }
  if (VERBOSE) {
    printf("Finished moving: %d, FR: %f, RR: %f, FL: %f, RL: %f\n", counter,
      right_drive.get_position(), right_rear_drive.get_position(),
      left_drive.get_position(), left_rear_drive.get_position());
  }
}

void a_move_relative(double target, std::int32_t velocity){
   move_by_distance(false, target, target, target, target, velocity, DEFAULT_TIMEOUT);;
 }

void a_move_relative(double target, std::int32_t velocity, int timeout){
   move_by_distance(false, target, target, target, target, velocity, timeout);;
}

void a_move_relative(double FR_target, double RR_target, double FL_target, double RL_target,
       std::int32_t velocity){
   move_by_distance(false, FR_target, RR_target, FL_target, RL_target, velocity, DEFAULT_TIMEOUT);
}

void a_move_relative(double FR_target, double RR_target, double FL_target, double RL_target,
      std::int32_t velocity, int timeout){
  move_by_distance(false, FR_target, RR_target, FL_target, RL_target, velocity, timeout);
}

void a_move_absolute(double position, std::int32_t velocity){
  move_by_distance(true, position, position, position, position, velocity, DEFAULT_TIMEOUT);
}

void a_move_absolute(double position, std::int32_t velocity, int timeout){
  move_by_distance(true, position, position, position, position, velocity, timeout);
}

void a_move_absolute(double FR_target, double RR_target, double FL_target, double RL_target,
    std::int32_t velocity){
  move_by_distance(true, FR_target, RR_target, FL_target, RL_target, velocity, DEFAULT_TIMEOUT);
}

void a_move_absolute(double FR_target, double RR_target, double FL_target, double RL_target,
    std::int32_t velocity, int timeout){
  move_by_distance(true, FR_target, RR_target, FL_target, RL_target, velocity, timeout);
}

void unfold() {
  a_move_hinge(127);
  delay(800);
  a_move_intake(-80);
  a_move_hinge(127);
  delay(100);
  a_move_intake(127);
  a_move_hinge(-127);
  delay(700);
  a_move_hinge(0);
}

void stackSixEight() {
  a_move_intake(0);
  a_move_lift(-20);

  a_move_hinge(100);
  a_move_intake(30);
  delay(900); //300

  a_move_hinge(60);
  a_move_intake(0);
  delay(400);

  a_move_hinge(60);
  a_move_intake(-60);
  delay(700);

  a_move_hinge(40);
  delay(1200);

  a_move_intake(0);
  a_move_drive(0, 0);
  a_move_hinge(0);
  a_move_lift(0);
}

void stackThreeFour() {
  a_move_intake(0);
  a_move_lift(-20);

  a_move_hinge(100);
  a_move_intake(20);
  delay(900); //300

  a_move_hinge(60);
  a_move_intake(-20);
  delay(1100);

  a_move_hinge(40);
  delay(1200);

  a_move_intake(0);
  a_move_drive(0, 0);
  a_move_hinge(0);
  a_move_lift(0);
}

/**
 * Autonomous program for one cube in any zone
 */
void one(bool isRed) {
  double ticks = -12;
  a_move_absolute(ticks, 80, 2000);
  ticks += 12;
  a_move_absolute(ticks, 80, 2000);
  a_move_drive(0, 0);
}

/**
 * Autonomous program for 3 cubes in the large zone.
 */
void large_three(bool isRed){
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();
  unfold();
  a_move_lift(-20);
  a_move_intake(127);
  double temp_ticks = 14;
  double RF_ticks = temp_ticks;
  double RR_ticks = temp_ticks;
  double LF_ticks = temp_ticks;
  double LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);
  temp_ticks = 7.5;
  if (isRed) { // For Red zone, turne left
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 1000);

  temp_ticks = 18;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 10000);

  temp_ticks = 3.95; //3.75
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);

  temp_ticks = 19;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 82, 6000);

  temp_ticks = -1.5;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 6000);

  stackThreeFour();

  temp_ticks = -12;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);

  a_move_hinge(-127);
  delay(2000);

  a_move_lift(0);
  a_move_hinge(0);
  a_move_intake(0);
  a_move_drive(0, 0);
}

/**
 * Autonomous program for 4 cubes in the large zone， without knocking down the stack.
 * Set the robot very close to the goal zone, facing the cube near goal zone
 * Route:
 * 0. preload
 * 1. Drive forward to pick up one cube close to the goal zone
 * 2. Turn towards small zone, drive to pick up the 3rd cube
 * 3. Slight turn, drive forward to pick up a cube by the tall tower
 * 4. Turn, drive back and score
 */
void large_four(bool isRed){
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  unfold();

  a_move_lift(-20);
  a_move_intake(127);

	// 1. Drive forward to pick up 2nd cube
	double temp_ticks = 14;
  double RF_ticks = temp_ticks;
  double RR_ticks = temp_ticks;
  double LF_ticks = temp_ticks;
  double LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);

	// 2. Turn 90 degrees.
  temp_ticks = 7.5;
  if (isRed) { // For Red zone, turne left
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 1000);

	// 3. Drive forward
  temp_ticks = 18;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 10000);

	// 4. Turn slightly
  temp_ticks = 3.95; //3.75
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);


  temp_ticks = 19;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 82, 6000);

  temp_ticks = -1.5;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 6000);

  stackThreeFour();

  temp_ticks = -12;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80, 3000);

  a_move_hinge(-127);
  delay(2000);

  a_move_lift(0);
  a_move_hinge(0);
  a_move_intake(0);
  a_move_drive(0, 0);
}

/**
 * @obsolete We can no longer use this program any more due to new rule change.
 * Autonomous program for 4 cubes in the large zone.
 */
void large_four_obsolete(bool isRed) {
    a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
    a_tare_position();
    unfold();
    a_move_lift(-20);
    a_move_intake(127);
    double ticks = 27; //31
    a_move_absolute(ticks, 110, 2400);
    delay(200);
    ticks += 19;
    a_move_absolute(ticks, 50);
    delay(300);
    //ticks += 4;
    //a_move_absolute(ticks, 50);
    //delay(300);
    ticks -= 45;
    a_move_absolute (ticks, 85, 600);
    ticks += 3.5;
    a_move_absolute (ticks, 75);
    double temp_ticks = 8.75;
    double RF_ticks = ticks;
    double RR_ticks = ticks;
    double LF_ticks = ticks;
    double LR_ticks = ticks;

    if (isRed) {
      RF_ticks += temp_ticks;
      RR_ticks += temp_ticks;
      LF_ticks -= temp_ticks;
      LR_ticks -= temp_ticks;
      a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1400);
    }
    else {
      RF_ticks -= temp_ticks;
      RR_ticks -= temp_ticks;
      LF_ticks += temp_ticks;
      LR_ticks += temp_ticks;
      a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1400);
    }
    a_move_intake(40);
    temp_ticks = 24; ////temp_ticks = 18.25;
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85);

    temp_ticks = 4; //temp_ticks = 7.5;
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30);
    stackThreeFour();

    temp_ticks = -12;
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 95);

    a_move_hinge(80);
    a_move_intake(-40);
    delay(1700);
    delay(300);

    a_move_relative(0, 60);
    a_move_hinge(0);
    a_move_intake(0);
}

/**
 * @absolete  No longer valid since we put the preload in the tray.
 * Auton program for 5 cubes in the small zone.
 */
void small_five(bool isRed) {
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  double ticks = 3;

  double RF_ticks = ticks;
  double RR_ticks = ticks;
  double LF_ticks = ticks;
  double LR_ticks = ticks;

  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75); ///

  double temp_ticks = 3; ///
  RF_ticks -= temp_ticks; ///
  RR_ticks -= temp_ticks; ///
  LF_ticks -= temp_ticks; ///
  LR_ticks -= temp_ticks; ///

  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75); ///

  unfold();

  a_move_lift(-20);
  a_move_intake(127);
  delay(200); //delay(400);

  temp_ticks = 28;

  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  delay(100);

  temp_ticks = 9;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30); //45
  delay(600); //delay 300

  a_move_lift(0);

  temp_ticks = -16;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2000);
  delay(100);

  temp_ticks = 11;

  if (isRed) {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1700);
  }
  else {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1700);
  }

  temp_ticks = 27; //temp_ticks = 21;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 45, 2000); //1800
  delay(30);

  a_move_intake(0);

  a_tare_position();

  temp_ticks = -1.5; //-1.75
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 70, 600);
  delay(30);

  lcd::print(2, "Before stacking. Timelap=%d\n", millis() - start_ts);

  a_move_intake(0);
  stackSixEight();
  a_move_intake(-45); //
  delay(200); //


  a_tare_position();

  a_move_intake(-40);
  a_move_drive(-100, -100);
  a_move_hinge(-127);
  delay(400);

  a_move_lift(0);
  a_move_intake(0);

  lcd::print(3, "After back off. Timelap=%d\n", millis() - start_ts);

  // Drive to position for manual control
  a_tare_position();

  temp_ticks = 10;

  if (isRed) {
    RF_ticks = temp_ticks;
    RR_ticks = temp_ticks;
    LF_ticks = (-1)*temp_ticks;
    LR_ticks = (-1)*temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks = (-1)*temp_ticks;
    RR_ticks = (-1)*temp_ticks;
    LF_ticks = temp_ticks;
    LR_ticks = temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  temp_ticks = -30;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;

  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2000);

  temp_ticks = 5;
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  a_move_hinge(0);
  a_move_drive(0, 0);
  a_move_intake(0);
}

/**
 * Auton program for 6 in small zone.
 */
void small_six(bool isRed) {
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  double ticks = 3; ///

  double RF_ticks = ticks; ///
  double RR_ticks = ticks; ///
  double LF_ticks = ticks; ///
  double LR_ticks = ticks; ///

  //a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75); ///

  double temp_ticks = 3; ///
  RF_ticks -= temp_ticks; ///
  RR_ticks -= temp_ticks; ///
  LF_ticks -= temp_ticks; ///
  LR_ticks -= temp_ticks; ///

  //a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75); ///

  a_move_hinge(127);
  delay(800);
  a_move_intake(-80);
  a_move_hinge(127);
  delay(100);
  a_move_intake(127);

  /*
  a_move_lift(-20);
  a_move_intake(127);
  delay(200); //delay(400);
  */
  a_move_hinge(-127);
  temp_ticks = 28;

  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  a_move_hinge(0);
  //delay(100);

  temp_ticks = 9;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30); //45
  delay(300); //delay 300

  temp_ticks = 2.5;

  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  temp_ticks = 8;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 55, 1200); //speed 75
  delay(600); //700

  temp_ticks = -8;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1200);

  temp_ticks = -4;
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 4000);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 4000);
  }

  a_move_lift(0);

  temp_ticks = -18;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 2000);
  delay(50);

  a_move_intake(80);

  if (isRed) {
    temp_ticks = 10.55; //11.25 //10
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }
  else {
    temp_ticks = 10.55; //11.55 //11.25
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }

  temp_ticks = 28.5; //temp_ticks = 21;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2050); //45 speed

  a_move_intake(0);

  a_tare_position();

  temp_ticks = -2.5; //-3
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 600);

  a_move_intake(-127);
  delay(150);
  stackSixEight();

  temp_ticks = -12;
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 120, 600);

  a_tare_position();
  /*
  a_move_intake(-40);
  a_move_drive(-100, -100);
  a_move_hinge(-127);
  delay(400);
  */
  a_move_hinge(-127);
  delay(1000);

  a_tare_position();
  /*
  temp_ticks = 10;
  if (isRed) {
    RF_ticks = temp_ticks;
    RR_ticks = temp_ticks;
    LF_ticks = (-1)*temp_ticks;
    LR_ticks = (-1)*temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1200);
  }
  else {
    RF_ticks = (-1)*temp_ticks;
    RR_ticks = (-1)*temp_ticks;
    LF_ticks = temp_ticks;
    LR_ticks = temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1200);
  }
  temp_ticks = -30;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1600);
  temp_ticks = 5;
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 240);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  */

  a_move_lift(0);
  a_move_hinge(0);
  a_move_drive(0, 0);
  a_move_intake(0);
}

void small_seven(bool isRed) {
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  unfold();

  a_move_lift(-20);
  a_move_intake(127);

  double temp_ticks = 24;
  double RF_ticks = 0;
  double RR_ticks = 0;
  double LF_ticks = 0;
  double LR_ticks = 0;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);

  temp_ticks = 9.25; //8.75
  if (isRed) {
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RR_ticks -= temp_ticks;
    RF_ticks -= temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);

  temp_ticks = -20;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2000);

  temp_ticks = 9.625;
  if (isRed) {
    RR_ticks -= temp_ticks;
    RF_ticks -= temp_ticks;
  }
  else {
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1000);

  temp_ticks = -3;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 600);

  a_tare_position();

  temp_ticks = 28;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  a_move_hinge(0);
  //delay(100);

  temp_ticks = 9;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30); //45
  delay(300); //delay 300

  a_move_lift(0);

  temp_ticks = -18;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 2000);
  delay(50);

  a_move_intake(80);

  if (isRed) {
    temp_ticks = 10.55; //11.25 //10
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }
  else {
    temp_ticks = 10.55; //11.55 //11.25
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }

  temp_ticks = 28.5; //temp_ticks = 21;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2050); //45 speed

  a_move_intake(0);

  a_tare_position();

  temp_ticks = -2.5; //-3
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 600);

  a_move_intake(-127);
  delay(150);
  stackSixEight();

  temp_ticks = -12;
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 120, 600);

  a_tare_position();

  a_move_hinge(-127);
  delay(1000);

  a_move_lift(0);
  a_move_hinge(0);
  a_move_drive(0, 0);
  a_move_intake(0);
}

/**
 * Skills:
 */
void skills_towers(bool isRed) {
  //small_six(isRed);

  a_tare_position();
  a_move_intake(127);

  double temp_ticks = -0.5;
  double RF_ticks = 0;
  double LF_ticks = 0;
  double RR_ticks = 0;
  double LR_ticks = 0;

  RF_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80);

  delay(400);

  a_tare_position();

  temp_ticks = 12;
  RF_ticks -= temp_ticks;
  RR_ticks -= temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 3200);

  temp_ticks = 28;
  RF_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80);

  delay(400);

  temp_ticks = -4;
  RF_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80);

    a_move_intake(-60);
    delay(600);
    a_move_intake(0);

  a_move_hinge(127);
  delay(300);
  a_move_hinge(0);
  a_move_lift(127);
  delay(1000);

  temp_ticks = 9.75;
  RF_ticks += temp_ticks;
    LF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 70);

  delay(200);

  a_move_intake(-90);
  delay(400);
  a_move_intake(0);
  /*
  temp_ticks = -10;
  RF_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 80);
  */
}

void skills_ten(bool isRed) {
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  double ticks = 3; ///

  double RF_ticks = ticks; ///
  double RR_ticks = ticks; ///
  double LF_ticks = ticks; ///
  double LR_ticks = ticks; ///

  double temp_ticks = 3; ///
  RF_ticks -= temp_ticks; ///
  RR_ticks -= temp_ticks; ///
  LF_ticks -= temp_ticks; ///
  LR_ticks -= temp_ticks; ///

  unfold();

  a_move_intake(127); //

  temp_ticks = 28;

  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  //delay(100);

  temp_ticks = 9;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30); //45
  delay(300); //delay 300

  temp_ticks = 0.5;

  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  temp_ticks = -0.5;

  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  temp_ticks = 38;

  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 65);

}

void skills_double(bool isRed) {
  a_set_drive_encoding(E_MOTOR_ENCODER_COUNTS);
  a_tare_position();

  unfold();

  a_move_lift(-20);
  a_move_intake(127);

  double temp_ticks = 24;
  double RF_ticks = 0;
  double RR_ticks = 0;
  double LF_ticks = 0;
  double LR_ticks = 0;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);

  temp_ticks = 8.75;
  if (isRed) {
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  else {
    RR_ticks -= temp_ticks;
    RF_ticks -= temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);

  temp_ticks = -23;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2000);

  temp_ticks = 9.625;
  if (isRed) {
    RR_ticks -= temp_ticks;
    RF_ticks -= temp_ticks;
  }
  else {
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
  }
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1000);

  temp_ticks = -3;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 600);

  a_tare_position();

  temp_ticks = 28;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  a_move_hinge(0);
  //delay(100);

  temp_ticks = 9;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 30); //45
  delay(300); //delay 300

  temp_ticks = 2.5;

  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75);
  }

  temp_ticks = 8;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 55, 1200); //speed 75
  delay(600); //700

  temp_ticks = -8;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 1200);

  temp_ticks = -4;
  if (isRed) {
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 4000);
  }
  else {
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 4000);
  }

  a_move_lift(0);

  temp_ticks = -18;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 2000);
  delay(50);

  a_move_intake(80);

  if (isRed) {
    temp_ticks = 10.55; //11.25 //10
    RF_ticks -= temp_ticks;
    RR_ticks -= temp_ticks;
    LF_ticks += temp_ticks;
    LR_ticks += temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }
  else {
    temp_ticks = 10.55; //11.55 //11.25
    RF_ticks += temp_ticks;
    RR_ticks += temp_ticks;
    LF_ticks -= temp_ticks;
    LR_ticks -= temp_ticks;
    a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 85, 1900);
  }

  temp_ticks = 28.5; //temp_ticks = 21;
  RF_ticks += temp_ticks;
  RR_ticks += temp_ticks;
  LF_ticks += temp_ticks;
  LR_ticks += temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 75, 2050); //45 speed

  a_move_intake(0);

  a_tare_position();

  temp_ticks = -2.5; //-3
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 127, 600);

  a_move_intake(-127);
  delay(150);
  stackSixEight();

  temp_ticks = -12;
  RF_ticks = temp_ticks;
  RR_ticks = temp_ticks;
  LF_ticks = temp_ticks;
  LR_ticks = temp_ticks;
  a_move_absolute(RF_ticks, RR_ticks, LF_ticks, LR_ticks, 120, 600);

  a_tare_position();

  a_move_hinge(-127);
  delay(1000);
}




void up_task_fn(void* hinge_down) {

	if ((* ((bool*)hinge_down)) == true) {
		a_move_hinge(127);
		delay(400);
		a_move_hinge(10);
		*((bool*)hinge_down) = false;
	}
	a_move_lift(127);
	delay(1550);
	a_move_lift(20);
	a_move_hinge(0);
}


void left_task_fn(void* hinge_down) {
	if ((* ((bool*)hinge_down)) == true) {
		a_move_hinge(127);
		delay(350);
		a_move_hinge(10);
		*((bool*)hinge_down) = false;
	}
	a_move_lift(127);
	delay(1000); //1100
	a_move_lift(20);

}

void right_task_fn(void* hinge_down) {
	if ((* ((bool*)hinge_down)) == true) {
		a_move_hinge(127);
		delay(350);
		a_move_hinge(10);
		*((bool*)hinge_down) = false;
	}
	a_move_lift(127);
	delay(750);
	a_move_lift(10);

}

void y_task_fn(void* hinge_down) {
	a_move_lift(-127); // Move down the lift.
	delay(250);
	a_move_hinge(-127);
	a_move_lift(-50);
	delay(200);//KS changed it from 150
	a_move_hinge(0);
	*((bool*)hinge_down) = true;//hinge_midway = true;//KS no midway
	a_move_lift(0);
	a_move_drive(0, 0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	/*
	 * Define variables used by different robot functionality
	 */
	// Drive code variables
	int left_x;  // Left joystick X
	int left_y;  // Left joystick Y
	int right_x; // Right joystick X
	int right_y; // Right joystick Y
	int right_power;
	int left_power;

	int button_l1;
	int button_l2;
	int button_r1;
	int button_r2;

	int button_up;
	int button_down;
	int button_left;
	int button_right;

	int button_x;
	int button_b;
	int button_y;
	int button_a;

	// A special variable to remember if the hinge is at the resting position.
	// When we press "A" to lift the arm, if the hinge is at the resting position,
	// we will move the hinge forward a little bit so that it does not block the arm.
	// However, if the hinge is not at the resting position, we do not need to
	// do anything speical.
	// NOTE: we assume whenever we move the hinge back, we always move all the way back.
	// This technically is not 100% accurate, but in reality, it should be the case.
	// Even if it is not accurate, the driver can always manully move the hinge.
	bool hinge_down = true;
	bool hinge_midway = false;

	left_drive.set_encoder_units(E_MOTOR_ENCODER_COUNTS);
	left_rear_drive.set_encoder_units(E_MOTOR_ENCODER_COUNTS);
	right_drive.set_encoder_units(E_MOTOR_ENCODER_COUNTS);
	right_rear_drive.set_encoder_units(E_MOTOR_ENCODER_COUNTS);
	left_drive.tare_position();
  left_rear_drive.tare_position();
  right_drive.tare_position();
  right_rear_drive.tare_position();


	while (true) {
		static int counter = 0;

		lcd::print(1, "2/29/2020 1:46PM, left intake port#=%d, counter=%d", LEFT_INTAKE_PORT, counter++);

		// Assign the values from the controller's joysticks to variables
		left_x = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		left_y = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		right_x = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		right_y = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		// filter out small moves by the button control
		if(std::abs(right_x) < DRIVE_THRESHOLD_RIGHT_CONTROL) {
			right_x = 0;
		}

		if (ARCADE) { // if arcade mode
			// Make the robot half sensitive to the turning signals
			right_x /= 2;

			right_power = left_y - right_x;
			left_power = left_y + right_x;
		}
		else { // if tank mode
			right_power = right_y;
			left_power = left_y;
		}

		// Limit motor powers so that they do not exceed the maximum motor possible power value
		if (std::abs(right_power) > 127) {
			right_power = std::copysign(127, right_power); // Copies the sign of right_power to 127
		}
		if (std::abs(left_power) > 127) {
			left_power = std::copysign(127, left_power); // Copies the sign of left_power to 127
		}

		// Limit drivetrain speed based on the constant defined above
		right_power = (int) (right_power * DRIVE_SPEED);
		left_power = (int) (left_power * DRIVE_SPEED);

		// Assign power to the motors
		a_move_drive(right_power, left_power);

		// **************************
		// L1: intake								R1: Manual move up hinge
		// L2: Manual unfold				R2: Manual move back hinge
		// 						Up: High tower
		// Left: Midle towers										Right: descore
		//						Down: Drive away after scoreing
		//
		//						X: Small stack scoring
		// Y:	Move hinge & arms to resting			A: Outtake
		//						B: Big stack scoring
		//
		// **************************

		// L1 button: Intake
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1) == 1) {
			a_move_lift(-20);

			a_move_intake(127);
			hinge_down = true;
		}

		// L2 button: manually unfold
		if (master.get_digital(E_CONTROLLER_DIGITAL_L2) == 1) {
			a_move_hinge(127);
			delay(800);
			a_move_intake(-80);
			a_move_hinge(127);
			delay(100);
			a_move_intake(127);
			a_move_hinge(-127);
			delay(700);
			a_move_hinge(0);
		}


		// R1 button: manually move up the hinge.
		 if (master.get_digital(E_CONTROLLER_DIGITAL_R1) == 1) {
 			lcd::print(4, "R1 pressed");
			a_move_intake(0); // Stop intake roller

 			a_move_hinge(50); // Move the lift forward
 			while (master.get_digital(E_CONTROLLER_DIGITAL_R1) == 1) { // Wait for the button to be released
 				delay(10);
 			}
 			a_move_hinge(0);
			delay(20);
 		}

		// R2 button: manually move down the hinge
		 if (master.get_digital(E_CONTROLLER_DIGITAL_R2) == 1) {
 			lcd::print(4, "R2 pressed");
 			a_move_intake(0);		// Stop intake roller
 			a_move_hinge(-127); // Move the hinge backward
 			while (master.get_digital(E_CONTROLLER_DIGITAL_R2) == 1) {
 				delay(10);
 			}
 			a_move_hinge(0);
			hinge_down = true;
 		}

		// Up button: Raise arm to the high tower
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP) == 1) {
			Task up_task(up_task_fn, &hinge_down, "Up Task");
		}

		// Left button: raise arm to the middle / alliance towers
		if (master.get_digital(E_CONTROLLER_DIGITAL_LEFT) == 1) {
			Task left_task(left_task_fn, &hinge_down, "Left Task");
		}

		// Right button: raise arms for descoring
		if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT) == 1) {
			Task right_task(right_task_fn, &hinge_down, "Right Task");
		}

		//Down button: Drive away while overtaking after scoring
		if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN) == 1) {
			int temp_ts = millis();
			a_move_intake(-40);
			a_move_drive(-40, -40);
			while (master.get_digital(E_CONTROLLER_DIGITAL_DOWN) == 1) {
				delay(10);
			}
			printf("Pressing Y for = %d millis\n", millis() - temp_ts);
			a_move_intake(0);
		}

		// Y button: Move hinge and arms to resting
		if (master.get_digital(E_CONTROLLER_DIGITAL_Y) == 1) {
			Task y_task(y_task_fn, &hinge_down, "Y Task");
		}

		// A button: outtake
		if (master.get_digital(E_CONTROLLER_DIGITAL_A) == 1) {
			a_move_intake(-100); // Move the intake belt backward, for removing blocks from towers
			while (master.get_digital(E_CONTROLLER_DIGITAL_A) == 1) {
				delay(10);
			}
			a_move_intake(0);
		}


		// X button: slow scoring
		if (master.get_digital(E_CONTROLLER_DIGITAL_X) == 1) {
			a_move_intake(0);
		  a_move_lift(-20);

		  a_move_hinge(100);
		  a_move_intake(20);
		  delay(900); //300

		  a_move_hinge(60);
		  a_move_intake(-20);
		  delay(1100);

		  a_move_hinge(40);
		  delay(1200);

		  a_move_intake(0);
		  a_move_drive(0, 0);
		  a_move_hinge(0);
		  a_move_lift(0);
		}

		// B button: Score a big stack
		if (master.get_digital(E_CONTROLLER_DIGITAL_B) == 1) {
			a_move_intake(0);
		  a_move_lift(-20);

		  a_move_hinge(100);
		  a_move_intake(80);
		  delay(800);

			a_move_hinge(100);
		  a_move_intake(70); //60
		  delay(200);

			a_move_hinge(100);
		  a_move_intake(0);
		  delay(700);

		  a_move_hinge(60);
		  a_move_intake(-20);
		  delay(400);

		  a_move_hinge(40);
		  delay(1200);

			a_move_intake(-127);
			a_move_hinge(0);
			delay(200);

			a_move_intake(0);
			a_move_drive(20, 20);
			delay(460);

		  a_move_intake(0);
		  a_move_drive(0, 0);
		  a_move_hinge(0);
		  a_move_lift(0);
		}

		delay(10);
	}
}

void autonomous() {
  redAlliance = false;
  autonNumber = 2;
  //1-small_six, 2-large_four, 3-skills
  start_ts = millis();

  switch (autonNumber) {
    case 0:
      small_seven(redAlliance);
      break;
    case 1:
      small_six(redAlliance);
      break;
    case 2:
      large_three(redAlliance);
      break;
    case 3:
      skills_towers(redAlliance);
      break;
    case 4:
      skills_ten(redAlliance);
      break;
    default:
      skills_double(redAlliance);
      break;
  }
}
