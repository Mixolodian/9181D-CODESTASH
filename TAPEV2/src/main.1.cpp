/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       115585/albert                                             */
/*    Created:      10/8/2024, 9:03:01 AM                                     */
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
motor LFM = motor(PORT11, ratio6_1,false);
motor LMM = motor(PORT12, ratio6_1,false);
motor LBM = motor(PORT13, ratio6_1,false);
motor RFM = motor(PORT2, ratio6_1,true);
motor RMM = motor(PORT3, ratio6_1,true);
motor RBM = motor(PORT4, ratio6_1,true);
//LFM stands for left front motor
motor INM = motor(PORT1, true);
motor INM2 = motor(PORT15, true);
//Intakes and misc motors

digital_out Clamp (Brain.ThreeWirePort.C);
digital_out Doinker (Brain.ThreeWirePort.A);

//starts the controller/names it
controller Controller;

motor_group LMG(LFM, LMM, LBM);
motor_group RMG(RFM, RMM, RBM);
motor_group IMG(INM, INM2);

drivetrain DriveTrain (LMG, RMG, 40, 350, 380);

void Autonomous(void){

  DriveTrain.setDriveVelocity(40,pct);
  DriveTrain.setTurnVelocity(35,pct);

  DriveTrain.driveFor(reverse,115,mm);
  DriveTrain.turnFor(right, 0.2, degrees);
  wait(0.5,sec);
  DriveTrain.setDriveVelocity(10,pct);
  wait(0.5,sec);
  DriveTrain.driveFor(reverse,34,mm);
  wait(0.5,sec);
  Clamp.set(true);
  wait(1,sec);
  IMG.spin(forward,100,pct);
    wait(0.5,sec);

  DriveTrain.setDriveVelocity(45,pct);
    wait(0.5,sec);
  DriveTrain.turnFor(left, 8, degrees);
    wait(0.5,sec);

  DriveTrain.driveFor(forward,130,mm);
    wait(0.5,sec);
   DriveTrain.setDriveVelocity(15,pct);
   wait(0.5,sec);
  DriveTrain.driveFor(forward,15,mm);
  wait(0.5,sec);

  DriveTrain.driveFor(reverse, 60, mm);
  wait(0.5,sec);
  DriveTrain.turnFor(left,4,degrees );
  wait(0.5,sec);
  
  DriveTrain.driveFor(forward,90,mm);
  wait(0.5,sec);
  DriveTrain.turnFor( left, 11,degrees);
  wait(0.5,sec);
  DriveTrain.driveFor(forward, 0,mm);
  wait(0.5,sec);

  Clamp.set(false);
  wait(0.5,sec);
  IMG.spin(forward,0,pct);




  

  

  
}



//sets up void function for motors
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


int main() {
    Brain.Screen.print(
      " >M< WATCH THEM SOCKS SANYA! "
    ); // Initializes robot configuration
    
    Clamp.set(false);

    Autonomous();

    while (true) {
      int forward = Controller.Axis3.position();
      int turn = Controller.Axis1.position();
// compensates for the bad friction
      int rightspeedadjustment = 0;
      
      int leftspeed = forward-turn;
      int rightspeed = forward+turn+rightspeedadjustment;

      int deadband = 0;

       
        coastFuntion(); // Set to coast mode when moving
        setDriveMotors(leftspeed, rightspeed); 
      

       ClampToggle();
       DoinkerToggle();
       
       IntakeStatus = false;

       intakecommand(); 
       setDriveMotors (leftspeed,rightspeed);
        

        this_thread::sleep_for(20);
    }
}





