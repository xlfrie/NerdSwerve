// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.CanController;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private ShuffleboardTab armTab = Shuffleboard.getTab("Arm Troubleshooting");
  private GenericEntry sbManualSpeedPercent = armTab.add("Speed Percent In", 0).withPosition(0, 0).getEntry();
  private GenericEntry sbManualTargetPosition = armTab.add("Target Position In", 0).withPosition(1, 0).getEntry();

  private GenericEntry sbkPIn = armTab.add("kP In ", 0).withPosition(0, 1).getEntry();
  private GenericEntry sbkIIn = armTab.add("kI In ", 0).withPosition(1, 1).getEntry();
  private GenericEntry sbkDIn = armTab.add("kD In ", 0).withPosition(2, 1).getEntry();
  private GenericEntry sbkIZoneIn = armTab.add("kIz In", 0).withPosition(3, 1).getEntry();
  private GenericEntry sbkFIn = armTab.add("kF In", 0).withPosition(4, 1).getEntry();
  private GenericEntry sbkMinOut = armTab.add("Min out", 0).withPosition(5, 1).getEntry();
  private GenericEntry sbkMaxOut = armTab.add("Max out", 0).withPosition(6, 1).getEntry();
  private GenericEntry sbConfigurePID = armTab.add("Configure PID", 0).withPosition(7, 1).getEntry();

  private GenericEntry sbkMaxVelocity = armTab.add("kMax velocity", 0).withPosition(0, 2).getEntry();
  private GenericEntry sbkMinVelocity = armTab.add("kMin velocity", 0).withPosition(1, 2).getEntry();
  private GenericEntry sbkMaxAcceleration = armTab.add("kMax Acceleration", 0).withPosition(2, 2).getEntry();
  private GenericEntry sbkAllowedError = armTab.add("kAllowed Error", 0).withPosition(3, 2).getEntry();
  private GenericEntry sbConfigureMagicMotion = armTab.add("Configure MM", 0).withPosition(4, 2).getEntry();


  private final CanController armLeftMotor;
  private final CanController armRightMotor;

  private final Encoder throughbore;


  private boolean enabled = SubSystemConfigs.kEnableArm;
  private double targetPosition = 0;  
  private boolean troubleshooting = true;
  private boolean smartMotionEnabled = false;

  private PIDController startUpPID;
  private boolean haventStarted = true;


  public Arm() {

    //double kMaxVelocity, double kMinVelocity, double kMaxAccel, double kAllowedError ){
           
    armLeftMotor = new CanController(ArmConstants.kLeftArmMotorID);
    armLeftMotor.configureMotor( ArmConstants.kRotorToSensorRatio, ArmConstants.kOpenLoopRampRate, ArmConstants.kControlFramePeriod, ArmConstants.kEncoderControlFramePeriod, ArmConstants.kLeftInverted, ArmConstants.kIdleBrake);
    armLeftMotor.configurePIDF(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kIz, ArmConstants.kF, ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
;    armLeftMotor.configureSmartMotion(ArmConstants.kMaxVelocity, ArmConstants.kMinVelocity, ArmConstants.kMaxAccel, ArmConstants.kAllowedError);

    armRightMotor = new CanController(ArmConstants.kRightArmMotorID);
    armRightMotor.configureMotor( ArmConstants.kRotorToSensorRatio, ArmConstants.kOpenLoopRampRate, ArmConstants.kControlFramePeriod, ArmConstants.kEncoderControlFramePeriod, ArmConstants.kRightInverted, ArmConstants.kIdleBrake);
    armRightMotor.configurePIDF(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kIz, ArmConstants.kF, ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
    armRightMotor.configureSmartMotion(ArmConstants.kMaxVelocity, ArmConstants.kMinVelocity, ArmConstants.kMaxAccel, ArmConstants.kAllowedError);


    startUpPID = new PIDController(0.0075, 0, 0);
    startUpPID.enableContinuousInput(0, 1);
    startUpPID.setTolerance(0.05);

    throughbore = new Encoder(7, 6);
    resetEncoder();


  }

  public void resetEncoder(){
    armLeftMotor.setPosition(0);
    armRightMotor.setPosition(0);
  }

  public void zeroaAbsoluteEncoder(){
    throughbore.reset();
  }

  

  public void stop(){
    armRightMotor.stop();
    armLeftMotor.stop();
  }



  @Override
  public void periodic() {





    if (getEnabled()) {
      if (smartMotionEnabled){
        armLeftMotor.setSmartMotionPositionControl(targetPosition);
        armRightMotor.setSmartMotionPositionControl(targetPosition);

      }
      else{
        armLeftMotor.setPositionControl(targetPosition);
        armRightMotor.setPositionControl(targetPosition);

      }
    } 
  
    else {
      if (troubleshooting){
        return;
      }
      else{
        stop();
      }
    }

    if (troubleshooting){
      if (sbConfigurePID.getDouble(0)==1){
        updatePIDFValueShuffleBoard();
      }

      if (sbConfigureMagicMotion.getDouble(0)==1){
       updateMagicMotionValueShuffleBoard(); 
      }
    }
    // This method will be called once per scheduler run
  }

   //****************************** STATE METHODS ******************************/
  public double getTargetPosition(){
    return targetPosition;
  }
  public void setTargetPosition(double pos){
    this.targetPosition = pos;


  }
  public double getPosition(){
    return armRightMotor.getPosition();
  }

  public double getAbsolutePosition() {
    return armLeftMotor.getAbsolutePosition();
  }

  public boolean hasReachedPosition(double position){
    if (Math.abs(getPosition()-position)<0.75){
      return true;
    }
    else{

      return false;
    }
  }
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
}

public boolean getEnabled(){
  return this.enabled;
}
  public void setEnabledSmartMotion(boolean enabled){
    this.smartMotionEnabled = enabled;
  }

  public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }
  public Command setEnabledCommand(boolean enabled) {
      return Commands.runOnce(() -> setEnabled(enabled));
  }

  public double getLeftAppliedVoltage(){
    return armLeftMotor.getMotorVoltage();
  }
    public double getRightAppliedVoltage(){
    return armRightMotor.getMotorVoltage();
  }

  public double getCurrentError(){
    return getTargetPosition()-getPosition();
  }

  public void incrementPosition(double increment) {
    setTargetPosition(getTargetPosition() + increment);   
}

  public Command incrementPositionCommand(double increment) {
    return Commands.runOnce(() -> incrementPosition(increment));
}

   //****************************** POSITION METHODS ******************************//

    
   public Command moveToNeutralCommand() {
    return Commands.runOnce(() -> setTargetPosition(ArmConstants.kNeutralPosition));
}
  public void moveToNeutral(){
  setTargetPosition(WristConstants.kNeutralPosition);
  }
  public void moveToGroundIntake(){
    setTargetPosition(ArmConstants.kGroundIntakePosition);
    setEnabled(true);
  }
  public Command moveToGroundIntakeCommand() {

    return Commands.runOnce(() -> moveToGroundIntake());
}

public void moveToShooterFeeding(){
  setTargetPosition(ArmConstants.kShooterFeedingPosition);
  setEnabled(true);
}

public Command moveToShooterFeedingCommand(){
      return Commands.runOnce(() -> moveToShooterFeeding());

}

public void moveToAmpScoring(){
    setTargetPosition(ArmConstants.kAmpScoringPosition);
    setEnabled(true);
  }
  public Command moveToAmpScoringCommand() {

    return Commands.runOnce(() -> moveToAmpScoring());
}

  //****************************** MANUAL METHODS ******************************//
public void setPercentage(double per){
  armLeftMotor.setSpeed(per);
 armRightMotor.setSpeed(per);
  setEnabled(false);





}

public void setPercentageShuffleBoard(){
  double speed = sbManualSpeedPercent.getDouble(0);
  setPercentage(speed);
  setEnabled(false);
}

public void setPositionShuffleBoard(){
  double targetPosition = sbManualTargetPosition.getDouble(0);
  setTargetPosition(targetPosition);
  setEnabled(true);
}


public void updatePIDFValueShuffleBoard(){

  //reserved for can
  double kP = sbkPIn.getDouble(0);
  double kI = sbkIIn.getDouble(0);
  double kD = sbkDIn.getDouble(0);
  double kIzone = sbkIZoneIn.getDouble(0);
  double kF = sbkFIn.getDouble(0);
  double kMinOut = sbkMinOut.getDouble(0);
  double kMaxOutput = sbkMaxOut.getDouble(0);
  
  armLeftMotor.configurePIDF(kP, kI, kD, kIzone, kF, kMinOut, kMaxOutput);
  armRightMotor.configurePIDF(kP, kI, kD, kIzone, kF, kMinOut, kMaxOutput);



 
}
public void updateMagicMotionValueShuffleBoard(){
  double kMaxVel = sbkMaxVelocity.getDouble(0);
  double kMinVel = sbkMinVelocity.getDouble(0);
  double kMaxAccel = sbkMaxAcceleration.getDouble(0);
  double kAllowedError = sbkAllowedError.getDouble(0);
  armLeftMotor.configureSmartMotion(kMaxVel, kMinVel, kMaxAccel, kAllowedError);
  armRightMotor.configureSmartMotion(kMaxVel, kMinVel, kMaxAccel, kAllowedError);
}

private boolean getHaventStarted(){
  return this.haventStarted;
}


public void initShuffleboard(int level) {
  //Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
  if (level == 0)  {
      return;
  }
  ShuffleboardTab tab = Shuffleboard.getTab("Arms");

  switch (level) {
      case 0:
        break;
      case 1:
   
        tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

      case 2:
        tab.addNumber("Absolute Position", () -> getAbsolutePosition());
        tab.addNumber("Current Error", () -> getCurrentError());
        tab.addBoolean("Enable Arm PID", () -> getEnabled());
        tab.addBoolean("Haven't started", () -> getHaventStarted());




      case 3:
        tab.addNumber("Left Turn percent (motor controller)", () -> getLeftAppliedVoltage());
        tab.addNumber("Right Turn percent (motor controller)", () -> getRightAppliedVoltage());
        tab.addNumber("Target Position", () -> getTargetPosition());
        tab.addNumber("Encoder Position", () -> getPosition());
        tab.addBoolean("At Target", () -> hasReachedPosition(getTargetPosition()));
  }
}

}


