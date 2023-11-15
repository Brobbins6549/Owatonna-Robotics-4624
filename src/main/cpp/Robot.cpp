// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Claw.h"
#include "Arm.h"
#include <math.h>

#include <fmt/core.h> 

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"
#include "commonauto/AutoSequence.h"
#include "commonauto/steps/WaitSeconds.h"
#include "commonauto/steps/TimeDriveHold.h"
#include "commonauto/steps/TurnToAbsoluteAngle.h"
#include "commonauto/steps/Stop.h"
#include "commonauto/steps/ResetNavXYaw.h"
#include "commonauto/steps/CalibrateNavXThenReset.h"
#include "commonauto/steps/MoveAndTurn.h"
#include "Auto/SetArm.h"
#include "Auto/SetClaw.h"
#include "Auto/SetClaw2.h"
#include "Auto/level.h"
#include "Auto/level2.h"
#include "Auto/level3.h"
#include "Auto/AutoPlan.h"

frc::Joystick* playerOne;
frc::XboxController* playerTwo;
AutoSequence* bigSequence;
frc::SendableChooser<std::string>* autoChooser;
frc::AnalogPotentiometer *m_ClawPotentiometer;
void Robot::RobotInit() {
playerOne = new frc::Joystick(R_controllerPortPlayerOne);
playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);
frc::SmartDashboard::PutNumber("Distance of the Arm Acuator", .35);
frc::SmartDashboard::PutNumber("Distance of the Claw Acuator", .15);
frc::CameraServer::StartAutomaticCapture();

autoChooser = new frc::SendableChooser<std::string>;
autoChooser->AddOption("move backwards", "move backwards");
autoChooser->AddOption("mid right", "mid right");
autoChooser->AddOption("level", "level");
autoChooser->AddOption("double left", "double left");
autoChooser->AddOption("double right", "double right");
autoChooser->AddOption("test triple","test triple");
autoChooser->AddOption("test double level","test double level");
autoChooser->AddOption("test movement", "test movement");
frc::SmartDashboard::PutData(autoChooser);

bigSequence = new AutoSequence(false);
bigSequence->EnableLogging();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
    SwerveTrain::GetInstance().ResetHold();
    SwerveTrain::GetInstance().HardwareZero();
    bigSequence->Reset();

    std::string selectedAuto = autoChooser->GetSelected();
    //Braylon's auto have been changed recently, ask what the auto does
    //and where it should start because it has been changed since of 4/15/23
    if (selectedAuto == "move backwards") {
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);

      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new WaitSeconds(.3));
      bigSequence->AddStep(new TimeDriveHold(0, -0.6, 3.65));

    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);

    bigSequence->AddStep(new WaitSeconds(20));
    }
    else if(selectedAuto == "mid right"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);

    bigSequence->AddStep(new TimeDriveHold(-.75, 0, 2));
    bigSequence->AddStep(new TimeDriveHold(0, -.5, 3.5));
    bigSequence->AddStep(new TimeDriveHold(.75, 0, 2));

      SwerveTrain::GetInstance().SetSwerveBrake(true);
      SwerveTrain::GetInstance().SetDriveBrake(true);

      bigSequence->AddStep(new WaitSeconds(20));
    }
    else if (selectedAuto == "level") {
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);

      bigSequence->AddStep(new SetArm(-.75));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new TurnToAbsoluteAngle(180));
      bigSequence->AddStep(new TimeDriveHold(0, -.5 , 4));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(90));
      bigSequence->AddStep(new TimeDriveHold(0, .6, 2.4));
      bigSequence->AddStep(new level2());
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new TimeDriveHold(.05, 0, .5));
   //   bigSequence->AddStep(new level2());
   //   bigSequence->AddStep(new level2());

  //    bigSequence->AddStep(new level2());
      bigSequence->AddStep(new WaitSeconds(20));
    }
    else if (selectedAuto == "double left") {
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new TurnToAbsoluteAngle(-15));
      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new TurnToAbsoluteAngle(185));
      bigSequence->AddStep(new WaitSeconds(.1));

      bigSequence->AddStep(new SetArm(.5));
      bigSequence->AddStep(new SetClaw(-.5));
      bigSequence->AddStep(new TimeDriveHold(0.08, -1, 2.7));

      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(.05));
      bigSequence->AddStep(new SetClaw(1));

      bigSequence->AddStep(new TurnToAbsoluteAngle(15));
      bigSequence->AddStep(new WaitSeconds(.15));
 
      bigSequence->AddStep(new SetClaw(0));   

      bigSequence->AddStep(new TimeDriveHold(-0.08, 1, 2.7));
      bigSequence->AddStep(new SetArm(-.75));

      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));
      bigSequence->AddStep(new TurnToAbsoluteAngle(180));

      bigSequence->AddStep(new TimeDriveHold(0, -0.5, 3.2));
      bigSequence->AddStep(new WaitSeconds(20));
    }
    else if (selectedAuto == "double right"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new TurnToAbsoluteAngle(15));
      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new TurnToAbsoluteAngle(185));
      bigSequence->AddStep(new WaitSeconds(.1));

      bigSequence->AddStep(new SetArm(.5));
      bigSequence->AddStep(new SetClaw(-.5));
      bigSequence->AddStep(new TimeDriveHold(-0.08, -1, 2.7));

      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(.05));
      bigSequence->AddStep(new SetClaw(1));

      bigSequence->AddStep(new TurnToAbsoluteAngle(-15));
      bigSequence->AddStep(new WaitSeconds(.15));

      bigSequence->AddStep(new SetClaw(0));   

      bigSequence->AddStep(new TimeDriveHold(0.08, 1, 2.7));
      bigSequence->AddStep(new SetArm(-.75));

      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));
      bigSequence->AddStep(new TurnToAbsoluteAngle(180));

      bigSequence->AddStep(new TimeDriveHold(0, -0.5, 3.2));
      bigSequence->AddStep(new WaitSeconds(20));

    }
    else if (selectedAuto == "test triple") {
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);


      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));
      bigSequence->AddStep(new WaitSeconds(.2));

      bigSequence->AddStep(new SetArm(.5));

      bigSequence->AddStep(new MoveAndTurn(0, -1, 185, 3.2, false));

      bigSequence->AddStep(new SetArm(.05));
      bigSequence->AddStep(new SetClaw(.5));
      bigSequence->AddStep(new MoveAndTurn(-0.1, 1, 15, 2.5, false));
      
      bigSequence->AddStep(new SetClaw(0)); 
      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));
      bigSequence->AddStep(new SetClaw(-.5));
      bigSequence->AddStep(new MoveAndTurn(0.05, -1, 270, 2.5, false));
      bigSequence->AddStep(new SetArm(.5));
      bigSequence->AddStep(new MoveAndTurn(0.5, 0, 270, 2.2, false));
      bigSequence->AddStep(new SetArm(0.05));
      bigSequence->AddStep(new MoveAndTurn(-0.5, 0, -15, 2.2, false));
      bigSequence->AddStep(new SetClaw(.5));
      bigSequence->AddStep(new MoveAndTurn(0, 1, -15, 2.5, false));
      bigSequence->AddStep(new SetArm(-.5));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));
      bigSequence->AddStep(new MoveAndTurn(0, -1, 180, 2.5, false));
      bigSequence->AddStep(new WaitSeconds(20));
    }
    else if (selectedAuto == "test double level"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);

      bigSequence->AddStep(new SetArm(-.75));
      bigSequence->AddStep(new WaitSeconds(.1));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new SetClaw(-.5));
      bigSequence->AddStep(new SetArm(.75));
      bigSequence->AddStep(new MoveAndTurn(0.04, -.5, 180, 4.3, false));

      bigSequence->AddStep(new SetArm(0.05));
      bigSequence->AddStep(new SetClaw(.5));
      bigSequence->AddStep(new MoveAndTurn(-0.04, .5, 0, 2.5, false));

      bigSequence->AddStep(new SetArm(.75));
      bigSequence->AddStep(new SetClaw(0));
      bigSequence->AddStep(new WaitSeconds(.2));
      bigSequence->AddStep(new SetArm(0));

      bigSequence->AddStep(new level2());
      bigSequence->AddStep(new WaitSeconds(20));
    }
    else if (selectedAuto == "test movement"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new ResetNavXYaw);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new MoveAndTurn(0, -1, 270, 3, false));
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new TurnToAbsoluteAngle(90));
      bigSequence->AddStep(new MoveAndTurn(1, 0, 0, 3, false));
    }
    bigSequence->AddStep(new Stop);
    bigSequence->Init();


}
void Robot::AutonomousPeriodic() {
  /* if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  } */
  bigSequence->Execute();
  
}

void Robot::TeleopInit() {
    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);
}

void Robot::TeleopPeriodic() {
  if (playerOne->GetRawButton(9) && playerOne->GetRawButton(10)) {

        SwerveTrain::GetInstance().HardwareZero();
    }
    if (playerOne->GetRawButton(4)) {

        NavX::GetInstance().Calibrate();
        NavX::GetInstance().resetYaw();
    }
    if (playerOne->GetRawButton(7)) {
        
        SwerveTrain::GetInstance().AssumeZeroPosition();
    }
    else {

        double x = playerOne->GetX();
        double y = playerOne->GetY();
        double z = playerOne->GetZ();
        Controller::forceControllerXYZToZeroInDeadzone(x, y, z);

        if (playerOne->GetRawButton(2)) {
            frc::SmartDashboard::PutNumber("z", z);
        }
        else {

            z *= R_controllerZMultiplier;
        }
//        if (playerOne->GetRawButton(11)) {
//          frc::SmartDashboard::PutNumber("Angle Value", NavX::GetInstance().getRoll());
//          double RollAngleRadius = NavX::GetInstance().getRoll() * (M_PI / 125);
//          SwerveTrain::GetInstance().Drive(0, (RollAngleRadius * -1), 0, false, false, 1); 
//        }
        SwerveTrain::GetInstance().Drive(
            -x,
            -y,
            z,
            playerOne->GetRawButton(1),
            playerOne->GetRawButton(3),
            -(((playerOne->GetThrottle() + 1.0) / 2.0) - 1.0)
        );
    }
/*
  if(playerTwo->GetLeftTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(.5 * playerTwo->GetLeftTriggerAxis());
    if(Arm::GetInstance().Current() > 19){
      playerTwo->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, .5);
    }
    else{
      playerTwo->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
    }

  }
  else if(playerTwo->GetRightTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(-(playerTwo->GetRightTriggerAxis()));
  }
  else{
    Arm::GetInstance().ArmSpeed(.05);
    playerTwo->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
  }

  if(playerTwo->GetRightBumper()){

    Claw::GetInstance().ClawTiltSpeed(-1); // claw tilt up
  }
  else if(playerTwo->GetLeftBumper()){
    Claw::GetInstance().ClawTiltSpeed(1); // claw tilt down
  }
  else{


    Claw::GetInstance().ClawTiltSpeed(0);
  }
*/
//Braylon's Controller Code (comment out if need other controller code (above) DO NOT DELETE, please and thank you :) )
  if (playerTwo->GetLeftTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(-(playerTwo->GetLeftTriggerAxis()));
    Claw::GetInstance().Idle(); // set claw to idle
  } else if (playerTwo->GetRightTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(playerTwo->GetRightTriggerAxis() * .55);
    Claw::GetInstance().ClawTiltPositionDown(.505);
    if (Arm::GetInstance().Current() > 20){
      playerTwo->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.5);

    }
  } else {
    Arm::GetInstance().ArmSpeed(.05);
    playerTwo->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
    Claw::GetInstance().ClawTiltPositionUp(.73);
  }

  Claw::GetInstance().Update();

}




void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  SwerveTrain::GetInstance().SetSwerveBrake(false);
  SwerveTrain::GetInstance().SetDriveBrake(false);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
