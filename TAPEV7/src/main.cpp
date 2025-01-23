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

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
//starts the brain
brain Brain;

//starts the motor ports
motor LFM = motor(PORT10, ratio6_1,true);
motor LMM = motor(PORT7, ratio6_1,true);
motor LBM = motor(PORT4, ratio6_1,true);
motor RFM = motor(PORT2, ratio6_1,false);
motor RMM = motor(PORT3, ratio6_1,false);
motor RBM = motor(PORT13, ratio6_1,false);
//LFM stands for left front motor
motor INM = motor(PORT19, false);
motor INM2 = motor(PORT6, true);
motor LDB = motor(PORT12, ratio18_1, true);
rotation LDR (PORT14);
optical Optical (PORT11);
inertial Inertial (PORT16);
//Intakes and misc motors

digital_out Clamp (Brain.ThreeWirePort.C);
digital_out Doinker (Brain.ThreeWirePort.A);

//starts the controller/names it
controller Controller;

motor_group LMG(LFM, LMM, LBM);
motor_group RMG(RFM, RMM, RBM);
motor_group IMG(INM, INM2);

drivetrain DriveTrain (LMG, RMG, 40, 350, 380);

const double wheelCircumference = 2.75 * M_PI; // 3.25" wheel diameter
const double ticksPerRevolution = 360.0;       // Motor encoder ticks per revolution
const double ticksToInches = wheelCircumference / ticksPerRevolution;


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
  Inertial.calibrate();
  LDR.resetPosition();
  while (Inertial.isCalibrating()){
    wait(1,sec);
  }
}

void DrivePID(double targetDistance, double targetHeading, double maxSpeed = 120, double turnScale = 0.5, double minSpeed = 5) {
    // PID constants for distance control
    double kP_distance = 1.5;
    double kI_distance = 0.01;
    double kD_distance = 0.2;

    // PID constants for heading control
    double kP_heading = 0.7;
    double kI_heading = 0.0;
    double kD_heading = 0.2;

    // Variables for distance PID
    double distanceError = 0;
    double distancePreviousError = 0;
    double distanceIntegral = 0;
    double distanceDerivative = 0;

    // Variables for heading PID
    double headingError = 0;
    double headingPreviousError = 0;
    double headingIntegral = 0;
    double headingDerivative = 0;

    // Reset motors and sensors
    LMG.resetPosition();
    RMG.resetPosition();
    Inertial.resetRotation();

    // PID control loop
    while (true) {
        // Get average motor position (distance traveled)
        double currentDistance = (LMG.position(degrees) + RMG.position(degrees)) / 2 * ticksToInches;

        // Distance PID calculations
        distanceError = targetDistance - currentDistance;
        distanceIntegral += distanceError; // Accumulate integral
        distanceDerivative = distanceError - distancePreviousError;

        double distancePower = (kP_distance * distanceError) +
                               (kI_distance * distanceIntegral) +
                               (kD_distance * distanceDerivative);

        // Constrain distance power to maxSpeed and apply minSpeed
        if (fabs(distancePower) < minSpeed && fabs(distanceError) > 0.5) {
            distancePower = copysign(minSpeed, distancePower); // Apply minimum speed with correct direction
        }
        if (distancePower > maxSpeed) distancePower = maxSpeed;
        if (distancePower < -maxSpeed) distancePower = -maxSpeed;

        distancePreviousError = distanceError;

        // Heading PID calculations
        double currentHeading = Inertial.rotation(degrees);
        headingError = targetHeading - currentHeading;
        headingIntegral += headingError;
        headingDerivative = headingError - headingPreviousError;

        double headingPower = (kP_heading * headingError) +
                              (kI_heading * headingIntegral) +
                              (kD_heading * headingDerivative);

        // Apply turn scaling to heading power
        headingPower *= turnScale;

        headingPreviousError = headingError;

        // Calculate motor speeds
        double leftSpeed = distancePower + headingPower;  // Left motors compensate for heading
        double rightSpeed = distancePower - headingPower; // Right motors compensate for heading

        // Set motor speeds
        LMG.spin(forward, leftSpeed, pct);
        RMG.spin(forward, rightSpeed, pct);

        // Break loop when bot
       if (fabs(distanceError) < 1 && fabs(headingError) < 1.5) {
            break;}

        wait(0.1,sec);
    }
    LMG.stop(brake);
    RMG.stop(brake);
}

// Drive command is DrivePID, ( Drive distance, turn degrees (its accurate so a 180 is a 180), drive speed, turn velocity(0.5 by default,),Minimun adjustment speed)
// IMG.spin or IMG.spinFor spins the intake, set for degrees or spinFor or just direction and velocity for spin
//LDB is the lady brown
// clamp.set is setting clamp on and off, true for on, false for off. 
void autonomous(void) {

LDB.setVelocity(90,pct); 
LDB.spinFor(reverse,360,degrees);
wait(0.5,sec);
DrivePID(-11,0,150);
wait(0.5,sec);
LDB.setVelocity(50,pct); 
DrivePID(0,90,100,0.5);
LDB.spinFor(forward,430,degrees,false);
DrivePID(-28,0,90);
Clamp.set(true);
LDB.stop(coast);
IMG.spin(forward,100,pct);
DrivePID(0,180,100,0.4);
DrivePID(45,0,80);
DrivePID(-35,0,100);
DrivePID(0,30);
DrivePID(20,0);
wait(0.3,sec);
DrivePID(0,180,100,0.6);
Clamp.set(false);
DrivePID(-14.9,0);

//first  lets go clamp

DrivePID(44,-20);
DrivePID(0,-180);
DrivePID(-70,0);
Clamp.set(true);
//second clamp is true
IMG.spin(forward,100,pct);
DrivePID(0,-102.5);
DrivePID(35,0);
DrivePID(0,-77);
DrivePID(35,0);
DrivePID(0,-100);
DrivePID(50,0);
DrivePID(-50,0);
DrivePID(0,-160);

/*DrivePID(33,10);
LDB.stop(hold);
LDB.spinFor(reverse,75,degrees);
DrivePID(0,85);
DrivePID(40,0);
IMG.setVelocity(100,pct);
IMG.spinFor(1800,degrees,false);

DrivePID(0,-19);
DrivePID(38,0);
DrivePID(-3,0);
DrivePID(0,80);
DrivePID(15,0);

IMG.spinFor(reverse,30,degrees);
LDB.spinFor(reverse,360,degrees,false);
wait(0.5,sec);
DriveTrain.driveFor(forward,10,mm);
DriveTrain.driveFor(reverse,5,mm);
IMG.spinFor(reverse,500,degrees);
DrivePID(-10,0);
LDB.spinFor(forward,400,degrees);
LDB.stop(coast);
IMG.spinFor(reverse,150,degrees);
//first wall stake
DrivePID(0,35);
wait(1,sec);
DrivePID(-85,7);
Clamp.set(true);
//2nd mogo clamped
IMG.spin(forward,100,pct);
DrivePID(0,-60);
DrivePID(25,0);
DrivePID(-25,0);*/

}

void DriverAlliance(){
  if (Controller.ButtonUp.pressing()){
    LDB.setVelocity(100,pct);
    LDB.spinFor(reverse,430,degrees);
    DriveTrain.driveFor(reverse,30,mm);
    wait(0.3,sec);
    LDB.spinFor(forward,430,degrees);
    LDB.stop(coast);
    wait(0.3,sec);
      }
}

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
void coastfunktion(){
  LFM.setBrake(coast);
  LMM.setBrake(coast);
  LBM.setBrake(coast);
  RFM.setBrake(coast);
  RMM.setBrake(coast);
  RBM.setBrake(coast);
}

bool IntakeStatus; 

double Hue = Optical.hue();
bool badcolour = false;
void intakecommand (){
  
  
  IntakeStatus = false;
  
  if (Controller.ButtonR1.pressing() && badcolour == false){
    INM.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeStatus = true;

   }

  if (Controller.ButtonR2.pressing()&& badcolour == false){
    INM.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeStatus = true;
   }
  if( IntakeStatus == false ){
    INM.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  }
   if (Controller.ButtonR2.pressing()&& Controller.ButtonB.pressing()){
    INM.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeStatus = true;
   }
   if (Controller.ButtonR1.pressing() && Controller.ButtonB.pressing()){
    INM.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    INM2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeStatus = true;

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


/*void ColourSorting(){
 Brain.Screen.printAt(80,10, "value:%2f",Hue);
 if (Hue<230 && Hue>210 && badcolour == false){
   badcolour = true;
    }
 if (!Optical.isNearObject() && badcolour == true){
  wait(0.8,sec);
   badcolour = false;
  
  }

 }*/
bool pidActive = false;       // Track whether the PID is active
double pidTarget = 0;         // Target angle for the PID
double pidError = 0;          // Current error
double pidPreviousError = 0;  // Previous error
double pidIntegral = 0;       // Integral term
double pidDerivative = 0;     // Derivative term

void LadyBrownPIDUpdate() {
    // PID constants
    double kP = 0.3;
    double kI = 0.01;
    double kD = 0.2;

    if (pidActive) {
        // Calculate error
        pidError = pidTarget - LDR.position(degrees);
        Brain.Screen.printAt(10,80,"Value: %2f",pidError);
        // Calculate integral
        if (fabs(pidError) < 20) {
            pidIntegral += pidError;
        } else {
            pidIntegral = 0;
        }

        // Calculate derivative
        pidDerivative = pidError - pidPreviousError;

        // Calculate motor speed
        double motorSpeed = ((kP * pidError) + (kI * pidIntegral) + (kD * pidDerivative))*1.8;

        // Constrain motor speed
        if (motorSpeed > 180) motorSpeed = 180;
        if (motorSpeed < -180) motorSpeed = -180;

        // Move motor
        LDB.spin(reverse, motorSpeed, pct);

        // Stop when the error is very small
        if (fabs(pidError) < 1) {
          LDB.stop(hold);
          Brain.Screen.print("FINISHED");
          pidActive = false; // PID is complete
        }

        // Update previous error
        pidPreviousError = pidError;
    }
}

void StartLadyBrownPID(double targetDegrees) {
    pidTarget = targetDegrees;
    pidError = pidTarget - LDR.position(degrees);
    pidPreviousError = pidError;
    pidIntegral = 0;
    pidDerivative = 0;
    pidActive = true; // Activate the PID controller
}

int LadyBrownStage = 1;

void usercontrol(void) {
  Clamp.set(false);
  Optical.setLight(ledState::on);
  Optical.setLightPower(100,pct);
  // User control code here, inside the loop
  while (1) {
    int forward = Controller.Axis3.position() *0.99;
    int turn = Controller.Axis1.position() * 0.50;
// compensates for the bad friction
    int rightspeedadjustment = 0;
    
    int leftspeed = forward+turn;
    int rightspeed = forward-turn+rightspeedadjustment;

    int deadband = 0;

    if (forward == 0 && turn == 0 ){
      brakefunction();
      }
      else{
        coastfunktion(); // Set to coast mode when moving
      setDriveMotors(leftspeed, rightspeed);
      }

      
      coastfunktion(); // Set to coast mode when moving
      setDriveMotors(leftspeed, rightspeed); 
    

      ClampToggle();
    
      IntakeStatus = false;

      intakecommand(); 
      DriverAlliance();
     //ColourSorting();
      setDriveMotors (leftspeed,rightspeed);
      
      if (Controller.ButtonA.pressing() && LadyBrownStage == 1 ) {
          StartLadyBrownPID(25);  
          LadyBrownStage = 2;
          wait(0.2,sec);
      }
      if (Controller.ButtonA.pressing() && LadyBrownStage == 2) {
          IMG.spinFor(reverse,20,degrees);
          StartLadyBrownPID(148);   
          LadyBrownStage = 3;
          wait(0.2,sec);
      }
      if (Controller.ButtonA.pressing() && LadyBrownStage == 3){
        StartLadyBrownPID(5);
        LadyBrownStage = 1;
        
        wait(0.2,sec);
      }
      Hue = Optical.hue();
      LadyBrownPIDUpdate();
      Optical.setLight(ledState::on);
      Optical.setLightPower(100,pct); // Continuously update PID calculations

      
    wait(3, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


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