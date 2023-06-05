#include "main.h"
#include <cmath>
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>
#include <tuple>
#include <algorithm>

PROS_USE_SIMPLE_NAMES;
using namespace pros;
using namespace std;
int firstLineStop;
/*
team: 2605E
version: v4.2
author: Cole J
*/
// port def
// motor (m)
Motor m_lf(11,E_MOTOR_GEARSET_06,false);
Motor m_lm(12,E_MOTOR_GEARSET_06,false);
Motor m_lb(13,E_MOTOR_GEARSET_06,false);
Motor_Group m_g_leftDrive({m_lf,m_lm,m_lb}); // left 6m drve group
Motor m_rf(14,E_MOTOR_GEARSET_06,true);
Motor m_rm(15,E_MOTOR_GEARSET_06,true);
Motor m_rb(16,E_MOTOR_GEARSET_06,true);
Motor_Group m_g_rightDrive({m_rf,m_rm,m_rb}); // right 6m drive group
Motor m_iri(19,E_MOTOR_GEARSET_06,true); // intake roller indexer
Motor m_fly(20,E_MOTOR_GEARSET_INVALID,true);
int v_drive = 127;
int v_iri = 127;
int v_fly = 127;
Motor_Group m_g_combinedDrive({m_rf,m_rm,m_rb,m_lf,m_lm,m_lb});
// sensor (s)
IMU s_imu_1(18);
IMU s_imu_2(17);
// 3 wire (w)
ADIDigitalOut w_expansionBlocker('d');
ADIDigitalOut w_expander('c');
ADIDigitalOut w_flyAngleChanger('b');
ADIDigitalIn w_AutoSelectButton('a');
// controller (c)
Controller c_master(CONTROLLER_MASTER);


double imuErrorCalc(double current, double target) { // current = s_imu_1.get_heading() target = std::fabs(360-a);
    double E1=target-current, E2=-1*(current-(target-360)), E3=target-(current-360), Efinal;

    if (fabs(E1) < fabs(E2) && fabs(E1) < fabs(E3)) {
      Efinal = E1;
    } 
    if (fabs(E2) < fabs(E1) && fabs(E2) < fabs(E3)) {
      Efinal = E2;
    } 
    if (fabs(E3) < fabs(E1) && fabs(E3) < fabs(E2)) {
      Efinal = E3;
    }
    return Efinal;
}

void autoFunction_drivePID(double arg_distance_d, double arg_kP, double arg_kI, double arg_kD, double arg_stopError) { 
  // pid forward drive function, arg_distance = distance traveled (inch), arg_velocity generally set to -1
  const double kP = arg_kP, kI = arg_kI, kD = arg_kD; // const (k), for PID tuning 
  double error, totalError = 0, prevError = 0, derivative;
  int int_error, int_prevError, int_error_count=0;
  // zeroing
  m_g_combinedDrive.set_zero_position(0.0);
  // inputs
  double currentPos = 0, desiredPos = (((300*arg_distance_d)/3.25)*-1)/2;
  // turn related inputs
  const double t_kP = 0, t_kI = 0.0, t_kD = 0.0;
  int t_error, t_totalError = 0, t_prevError = 0, t_derivative;
  double currentAngle, desiredAngle = s_imu_1.get_heading();
  do {
                        // drive section
    // sensor
    currentPos = (m_lf.get_position() + m_rf.get_position() + m_lm.get_position() + m_rm.get_position() + m_lb.get_position() + m_rb.get_position())/6; 
    
    // vertical drive pid
    // p
    error = currentPos - desiredPos;
    // i
    totalError += error;
    // d
    derivative = error - prevError;
    // pid velocity
    int velocity_l = error * kP + totalError * kI + derivative * kD;
                        // turn section
    // turn pid / straight drive
    // p
    t_error = imuErrorCalc(s_imu_1.get_heading(),desiredAngle);
    // i
    t_totalError += t_error;
    // d
    t_derivative = t_error - t_prevError;
    // pid velocity
    int turn_velocity_l = t_error * t_kP + t_totalError * t_kI + t_derivative * t_kD;
    // preverror setting
    t_prevError = t_error;
    // setting move velocity with turn velocity
    m_g_leftDrive.move((velocity_l-turn_velocity_l)*-1);
    m_g_rightDrive.move((velocity_l+turn_velocity_l)*-1);
    // pre term
    int_error = error;
    int_prevError = prevError;

    cout<<error<<endl;

    delay(20);
    prevError = error;

  } while (std::fabs(error) > arg_stopError);
  m_g_combinedDrive.brake();
}

void autoFunction_turnPID(double arg_turnDegree, double arg_kP, double arg_kI, double arg_kD, double arg_stopError, int arg_loopTermNum) { // , double arg_constantArray[] 
  // pid turn drive function, arg_turnDegree = degrees turned, imu start set to 180 arg_velocity generally set to -1
  //const double kP = arg_constantArray[0], kI = arg_constantArray[1], kD = arg_constantArray[2]; // const (k), for PID tuning
  
  const double kP = arg_kP, kI = arg_kI, kD = arg_kD;

  arg_turnDegree += s_imu_1.get_heading();
  
  double error, totalError = 0, prevError = 0, derivative;
  int int_error, int_prevError, int_error_count=0;

  do {
    // turn pid
    // p
    error = imuErrorCalc(s_imu_1.get_heading(),arg_turnDegree);
    // i
    totalError += error;
    // d
    derivative = error - prevError;
    // pid velocity
    int velocity_l = error * kP + totalError * kI + derivative * kD;

    m_g_leftDrive.move(velocity_l*-1);
    m_g_rightDrive.move(velocity_l);

    cout<<error<<endl;


    /*
    int_error = error;
    int_prevError = prevError;
    if (int_prevError == int_error) {
      int_error_count += 1;
    } else {
      int_error_count = 0;
    }
    if (int_error_count > arg_loopTermNum) {
      error = arg_stopError - .01;
    }*/


    delay(20);
    prevError = error;


  } while (std::fabs(error) > arg_stopError);
  m_g_combinedDrive.brake();
}

bool flyPID_activateFlyPID_b = false;
bool flyPID_atTargetVelocity_b = false;
int flyPID_targetVelocity_i = 0;
int flyPID_targetRange = 10;

void taskFunction_flyPID() {
  const double kP = 9.8, kI = 0.049, kD = 0.015; // 5.9 .003 .0002 / 6 .1 .2
  double currentVelocity = m_fly.get_actual_velocity(), desiredVelocity = flyPID_targetVelocity_i/18;
  double error, totalError = 0, prevError = 0, derivative;
  while(true) {
    if (flyPID_activateFlyPID_b) {
      // sensor
      currentVelocity = std::fabs(m_fly.get_actual_velocity());
      desiredVelocity = flyPID_targetVelocity_i/18;
      // p
      error = currentVelocity - desiredVelocity;
      // i
      totalError += error;
      // d
      derivative = error - prevError;
      // pid velocity
      int velocity_l = error * kP + totalError * kI + derivative * kD;
      // seting motor velocity
      m_fly.move(velocity_l*-1);

      //cout<<m_fly.get_actual_velocity()*18<<endl;
      
      if (std::fabs(error) < flyPID_targetRange) {
        flyPID_atTargetVelocity_b = true;
      } else {
        flyPID_atTargetVelocity_b = false;
      }
      cout<<flyPID_atTargetVelocity_b<<endl;
      //cout<<fabs(error)<<endl;

      /*
      // cout
      std::cout<<flyPID_atTargetVelocity_b;
      std::cout<<" e";std::cout<<std::fabs(error)<<std::endl;
      //std::cout<<" cv";std::cout<<currentVelocity;std::cout<<" t";std::cout<<desiredVelocity<<std::endl;
      */
    } else {
      m_fly.brake();
    }
    delay(20);
  }
}

void autoFunction_disk(int arg_diskShotNum, double arg_iriMoveConst) {
  for (int i = 0; i < arg_diskShotNum; i++) {
    while(!(flyPID_atTargetVelocity_b)) {
      delay(100); 
      m_iri.brake();
    }

    m_iri.set_zero_position(0.0);
    m_iri.move(v_iri*-10);

    while(m_iri.get_position() > -240) {
      delay(3);
    }
    cout<<m_iri.get_position()<<endl;
    m_iri.brake();
    delay(500); 
  }
}

void autoFunction_roller(int arg_movePos_negativeValue) {
  m_iri.set_zero_position(0.0);
  m_iri.move_absolute(arg_movePos_negativeValue,v_iri*2);
  while(m_iri.get_position() < arg_movePos_negativeValue) {
    delay(5);
  }
  m_iri.brake();
}

bool runTask_iri = false;
void taskFunction_runIri() {
  if (runTask_iri) {
    m_iri.move(v_iri);
  } else {
    m_iri.brake();
  }
}

bool timer_active = false;
int timer_seconds = 0;

void taskFunction_timer() {
  while(timer_active) {
    delay(1000);
    timer_seconds += 1;
  }
}

int selectedAuto = 0;
std::string autodescription_sa[9] = {"skills", "left 1r", "right 1r", "no auto"};
void buttonFunction_btn0_callback() {
  selectedAuto += 1;
  if(selectedAuto > 3) {
    selectedAuto = 0;
  }
}
void taskFunction_lcdHandler() {
  while(true) {
    std::string text_local = autodescription_sa[selectedAuto];
    lcd::set_text(1,"2605E, fw:v4.2, lcd auto select:");
    lcd::set_text(2,text_local);
	c_master.set_text(0,0,text_local);
    if ((w_AutoSelectButton.get_new_press()) || (c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))) {
      buttonFunction_btn0_callback();
    }
    delay(50);
  }
}
int iri_const = 1;
bool iri_active_b = false;
bool flyangletemp = false;
// primary functions
void initialize() {

  
  
  // lcd
  lcd::initialize();
  // tasks
  Task task_flyPID(taskFunction_flyPID);
  Task task_intake(taskFunction_runIri);
  Task task_lcdHandler(taskFunction_lcdHandler);
  Task task_timer(taskFunction_timer);
  // motor setup
  m_g_combinedDrive.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  m_iri.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  m_iri.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void disabled() {}
void competition_initialize() {}
void autonomous() {
	switch(selectedAuto) {
		case 0: // skills
		break;
		case 1: // left 1r
    autoFunction_drivePID(-1, 3,.0001,.00002,10);
    autoFunction_roller(800);
		break;
		case 2: // right 1r
    autoFunction_drivePID(-22,.2,.0001,.00001,5);
    autoFunction_turnPID(90,1.5,.007,.0001,1,100000);
    autoFunction_drivePID(-9,.5,.0002,.00001,10);
    autoFunction_roller(800);
		break;
    case 3:
    break;
	}
}

bool expanderBool = false;
bool expansionBlockerBool = false;

void opcontrol() {
  
  flyPID_activateFlyPID_b = true;
  while(true) {
    //cout<<flyangletemp<<endl;
    if(c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      flyangletemp = (flyangletemp) ? false : true;
      w_flyAngleChanger.set_value(flyangletemp);
    }
    if (flyangletemp) {
      flyPID_targetVelocity_i = 3200; // angle down
    } else {
      flyPID_targetVelocity_i = 2750; // angle up
    }
    // intake roller setting
    if(c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      iri_active_b = (iri_active_b) ? false : true;
    }
    // indexer setting
    if(c_master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      iri_const = -1;
    } else {
      iri_const = 1;
    }
    // iri move
    if(iri_active_b == true || iri_const == -1) {
      //m_iri.move(v_iri*iri_const);
      if (iri_const == -1) {
        m_iri.move(v_iri*-1*.9);
      } else {
        m_iri.move(v_iri);
      }
    } else {
      m_iri.brake();
    } 
    // fly setting
    if(c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      flyPID_activateFlyPID_b = (flyPID_activateFlyPID_b) ? false : true; 
    }
    // expander
    if(c_master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      expanderBool = (expanderBool) ? false : true;
      w_expander.set_value(expanderBool);
    }
    if(c_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      expansionBlockerBool = (expansionBlockerBool) ? false : true;
      w_expansionBlocker.set_value(expansionBlockerBool);
    }
    // drive
    int c_master_anaX = c_master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
    int c_master_anaY = c_master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y); 
    m_g_leftDrive.move((-1*c_master_anaY-c_master_anaX)*1);
    m_g_rightDrive.move((-1*c_master_anaY+c_master_anaX)*1);
    delay(20);
  }
}