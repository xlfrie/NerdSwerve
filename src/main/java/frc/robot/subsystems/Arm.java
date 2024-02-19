// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.CanController;
import frc.robot.utils.TalonFXController;

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

  private final DutyCycleEncoder throughBore;


  private boolean enabled = SubSystemConfigs.kEnableShooter;
  private double targetPosition = 0;  
  private boolean troubleshooting = true;
  private boolean smartMotionEnabled = true;


  public Arm() {
    armLeftMotor = new CanController(0);
    armLeftMotor.configureMotor( IntakeConstants.kRotorToSensorRatio, IntakeConstants.kOpenLoopRampRate, IntakeConstants.kControlFramePeriod, IntakeConstants.kEncoderControlFramePeriod, IntakeConstants.kInverted, IntakeConstants.kIdleBrake);
    armLeftMotor.configurePIDF(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);
    armLeftMotor.configureSmartMotion(targetPosition, targetPosition, targetPosition, targetPosition);

    armRightMotor = new CanController(0);
    armRightMotor.configureMotor( IntakeConstants.kRotorToSensorRatio, IntakeConstants.kOpenLoopRampRate, IntakeConstants.kControlFramePeriod, IntakeConstants.kEncoderControlFramePeriod, IntakeConstants.kInverted, IntakeConstants.kIdleBrake);
    armRightMotor.configurePIDF(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);
    armRightMotor.configureSmartMotion(targetPosition, targetPosition, targetPosition, targetPosition);

    throughBore = new DutyCycleEncoder(0);
    resetEncoder();
  }

  public void resetEncoder(){
    throughBore.setPositionOffset(WristConstants.kEncoderOffset);    
    armLeftMotor.setPosition(getAbsolutePosition());
    armRightMotor.setPosition(getAbsolutePosition());
  }

  
  public void zeroAbsoluteEncoder(){
    throughBore.setPositionOffset(throughBore.getAbsolutePosition());
  }

  public void stop(){
    armRightMotor.stop();
    armLeftMotor.stop();
  }



  @Override
  public void periodic() {

    if (enabled) {
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
      if (sbConfigurePID.getBoolean(false)){
        updatePIDFValueShuffleBoard();
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
    return throughBore.getAbsolutePosition() - throughBore.getPositionOffset();
  }

  public boolean hasReachedPosition(double position){
    if (Math.abs(getAbsolutePosition()-position)<WristConstants.kAllowedError){
      return true;
    }
    else{
      return false;
    }
  }
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
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
    return Commands.runOnce(() -> setTargetPosition(WristConstants.kStowPosition));
}
public void moveToNeutral(){
setTargetPosition(WristConstants.kStowPosition);
}

  //****************************** MANUAL METHODS ******************************//
public void setPercentage(double per){
  armLeftMotor.setSpeed(per);
  armRightMotor.setSpeed(per);

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


    //double kMaxVelocity, double kMinVelocity, double kMaxAccel, double kAllowedError

  double kMaxVel = sbkMaxVelocity.getDouble(0);
  double kMinVel = sbkMinVelocity.getDouble(0);
  double kMaxAccel = sbkMaxAcceleration.getDouble(0);
  double kAllowedError = sbkAllowedError.getDouble(0);
  armLeftMotor.configureSmartMotion(kMaxVel, kMinVel, kMaxAccel, kAllowedError);
  armRightMotor.configureSmartMotion(kMaxVel, kMinVel, kMaxAccel, kAllowedError);
}

public void initShuffleboard(int level) {
  //Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
  if (level == 0)  {
      return;
  }
  ShuffleboardTab tab = Shuffleboard.getTab("Wrist");

  switch (level) {
      case 0:
        break;
      case 1:
   
        tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

      case 2:
        tab.addNumber("Absolute Position", () -> getAbsolutePosition());
        tab.addNumber("Current Error", () -> getCurrentError());

      case 3:
        tab.addNumber("Left Turn percent (motor controller)", () -> getLeftAppliedVoltage());
        tab.addNumber("Right Turn percent (motor controller)", () -> getRightAppliedVoltage());
        tab.addNumber("Target Position", () -> getTargetPosition());
        tab.addNumber("Encoder Position", () -> getPosition());
        tab.addBoolean("At Target", () -> hasReachedPosition(getTargetPosition()));
  }
}

}


