/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       alber                                                     */
/*    Created:      1/20/2025, 8:41:11 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
//starts the brain
brain Brain;

//starts the motor ports
motor LFM = motor(PORT10, ratio6_1,true);
motor LMM = motor(PORT8, ratio6_1,true);
motor LBM = motor(PORT4, ratio6_1,true);
motor RFM = motor(PORT2, ratio6_1,false);
motor RMM = motor(PORT3, ratio6_1,false);
motor RBM = motor(PORT1, ratio6_1,false);
//LFM stands for left front motor
motor INM = motor(PORT7, true);
motor INM2 = motor(PORT6, false);
motor LDB = motor(PORT12, ratio18_1, true);
//Intakes and misc motors

digital_out Clamp (Brain.ThreeWirePort.C);
digital_out Doinker (Brain.ThreeWirePort.A);

//starts the controller/names it
controller Controller;

motor_group LMG(LFM, LMM, LBM);
motor_group RMG(RFM, RMM, RBM);
motor_group IMG(INM, INM2);

drivetrain DriveTrain (LMG, RMG, 40, 350, 380);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



void setDriveMotors(int leftspeed, int rightspeed) {
    LFM.spin(forward, leftspeed, pct);
    LMM.spin(forward, leftspeed, pct);
    LBM.spin(forward, leftspeed, pct);
    RFM.spin(forward, rightspeed, pct);
    RMM.spin(forward, rightspeed, pct);
    RBM.spin(forward, rightspeed, pct);

}
//Sets the Brake funtion
void brakefunction(){
  LFM.setBrake(brake);
  LMM.setBrake(brake);
  LBM.setBrake(brake);
  RFM.setBrake(brake);
  RMM.setBrake(brake);
  RBM.setBrake(brake);
}

//Sets the Coast funtion
void coastFuntion(){
  LFM.setBrake(coast);
  LMM.setBrake(coast);
  LBM.setBrake(coast);
  RFM.setBrake(coast);
  RMM.setBrake(coast);
  RBM.setBrake(coast);
}

bool IntakeStatus; 

void intakecommand (){
  
  
  IntakeStatus = false;
  
  if (Controller.ButtonR1.pressing()){
    INM.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeStatus = true;

   }

  if (Controller.ButtonR2.pressing()){
    INM.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeStatus = true;
   }
  if( IntakeStatus == false ){
    INM.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  }


  
}

bool ClampState = false;



void ClampToggle(){

  if (Controller.ButtonL2.pressing()){
     ClampState = true;
    }
  if (Controller.ButtonL1.pressing()){
    ClampState = false;
  }
  Clamp.set(ClampState);
  
}

bool DoinkerState = false;



void DoinkerToggle(){

  if (Controller.ButtonX.pressing()){
     DoinkerState = true;
    }
  if (Controller.ButtonA.pressing()){
    DoinkerState = false;
  }
  Doinker.set(DoinkerState);
  
}

void LadyBrown (){
  LDB.setStopping(hold);
  if(Controller.ButtonA.pressing()){
    LDB.spin(forward,40,pct);
  }
  else if(Controller.ButtonB.pressing()){
    LDB.spin(reverse,40,pct);
  }
  else if(Controller.ButtonX.pressing()){
    LDB.spinFor(forward,35,degrees);
    wait(0.8,sec);
  }
  else {
    LDB.spin(reverse,0,pct);
  }
}



void usercontrol(void) {
  Clamp.set(false);
  // User control code here, inside the loop
  while (1) {
    int forward = Controller.Axis3.position() *0.99;
      int turn = Controller.Axis1.position() * 0.7;
// compensates for the bad friction
      int rightspeedadjustment = 0;
      
      int leftspeed = forward+turn;
      int rightspeed = forward-turn+rightspeedadjustment;

      int deadband = 0;

      if (forward == 0 && turn == 0 ){
        brakefunction();
        }
        else{
          coastFuntion(); // Set to coast mode when moving
        setDriveMotors(leftspeed, rightspeed);
        }

       
        coastFuntion(); // Set to coast mode when moving
        setDriveMotors(leftspeed, rightspeed); 
      

       ClampToggle();
       DoinkerToggle();
       LadyBrown();
       
       IntakeStatus = false;

       intakecommand(); 
       setDriveMotors (leftspeed,rightspeed);
        
    
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}