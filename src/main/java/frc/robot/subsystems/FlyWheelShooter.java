// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.CanController;

public class FlyWheelShooter extends SubsystemBase {
  /** Creates a new FlyWheelShooter. */

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Troubleshooting");
  private GenericEntry sbManualTopSpeedPercent = shooterTab.add("Top Speed Percent In", 0).withPosition(0, 0).getEntry();
  private GenericEntry sbManualBottomSpeedPercent = shooterTab.add("Bottom Speed Percent In", 0).withPosition(1, 0).getEntry();
  private GenericEntry sbManualTargetVelocity = shooterTab.add("Target Velocity In", 0).withPosition(2, 0).getEntry();
  
  private GenericEntry sbkPIn = shooterTab.add("kP In ", 0).withPosition(0, 1).getEntry();
  private GenericEntry sbkIIn = shooterTab.add("kI In ", 0).withPosition(1, 1).getEntry();
  private GenericEntry sbkDIn = shooterTab.add("kD In ", 0).withPosition(2, 1).getEntry();
  private GenericEntry sbkIZoneIn = shooterTab.add("kIz In", 0).withPosition(3, 1).getEntry();
  private GenericEntry sbkFIn = shooterTab.add("kF In", 0).withPosition(4, 1).getEntry();
  private GenericEntry sbkMinOut = shooterTab.add("Min out", 0).withPosition(5, 1).getEntry();
  private GenericEntry sbkMaxOut = shooterTab.add("Max out", 0).withPosition(6, 1).getEntry();

  private GenericEntry sbkMaxVelocity = shooterTab.add("Max Velocity ", 0).withPosition(0, 2).getEntry();
  private GenericEntry sbkMinVelocity = shooterTab.add("Min Velocity", 0).withPosition(1, 2).getEntry();
  private GenericEntry sbkMaxAccel = shooterTab.add("Max Accel", 0).withPosition(2, 2).getEntry();
  private GenericEntry sbkAllowedError = shooterTab.add("Allowed Error", 0).withPosition(3, 2).getEntry();
  private GenericEntry sbConfigurePID = shooterTab.add("Configure PID", 0).withPosition(4, 2).getEntry();


  CanController bottomMotor;
  CanController topMotor;

  private boolean enabled = SubSystemConfigs.kEnableShooter;
  private double targetVelocity = 0;

  private boolean troubleshooting = true;


  public FlyWheelShooter() {

    bottomMotor = new CanController(FlywheelConstants.kBottomShooterID);
    topMotor = new CanController(FlywheelConstants.kTopShooterID);

    bottomMotor.configureMotor( FlywheelConstants.kRotorToSensorRatio, FlywheelConstants.kOpenLoopRampRate, FlywheelConstants.kControlFramePeriod, FlywheelConstants.kEncoderControlFramePeriod, FlywheelConstants.kInverted, FlywheelConstants.kIdleBrake);
    bottomMotor.configurePIDF(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD, FlywheelConstants.kIz, FlywheelConstants.kF, FlywheelConstants.kMinOutput, FlywheelConstants.kMaxOutput);

    topMotor.configureMotor(FlywheelConstants.kRotorToSensorRatio, FlywheelConstants.kOpenLoopRampRate, FlywheelConstants.kControlFramePeriod, FlywheelConstants.kEncoderControlFramePeriod, FlywheelConstants.kInverted, FlywheelConstants.kIdleBrake);
    topMotor.configurePIDF(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD, FlywheelConstants.kIz, FlywheelConstants.kF, FlywheelConstants.kMinOutput, FlywheelConstants.kMaxOutput);

  }

    public void resetEncoder(double pos){
    bottomMotor.setPosition(pos);
    topMotor.setPosition(pos);
  }

  public void stop(){
    topMotor.stop();
    bottomMotor.stop();
  }

  @Override
  public void periodic() {

    if (enabled) {
      topMotor.setVelocityControl(targetVelocity);
      bottomMotor.setVelocityControl(targetVelocity);
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
    }
    // This method will be called once per scheduler run
  }


     //****************************** STATE METHODS ******************************/
  public double getTargetVelocity(){
    return targetVelocity;
  }
  public void setTargetVelocity(double vel){
    this.targetVelocity = vel;

  }
  
  public double getTopVelocity(){
    return topMotor.getVelocity();
  }

  public double getBottomVelocity(){
    return bottomMotor.getVelocity();
  }

    public double getVelocity(){
    return (bottomMotor.getVelocity()+topMotor.getVelocity())/2;
  }


  public boolean hasReachedVelocity(double velocity){
    if (Math.abs(getVelocity()-velocity)<FlywheelConstants.kAllowedError){
      return true;
    }
    else{
      return false;
    }
  }
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
}


  public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }
  public Command setEnabledCommand(boolean enabled) {
      return Commands.runOnce(() -> setEnabled(enabled));
  }

  public double getTopAppliedVoltage(){
    return topMotor.getMotorVoltage();
  }

  public double getBottomAppliedVoltage(){
    return bottomMotor.getMotorVoltage();
  }


  public double getCurrentError(){
    return getTargetVelocity()-getVelocity();
  }

   public double getCurrentErrorPercentage(){
    return (getTargetVelocity()-getVelocity())/(getTargetVelocity());
  }

  public void incrementVelocity(double increment) {
    setTargetVelocity(getTargetVelocity() + increment);   
}

public Command incrementVelocityCommand(double increment) {
    return Commands.runOnce(() -> incrementVelocity(increment));
}


      //****************************** VELOCITY METHODS ******************************//

    
public Command speedToNeutralCommand() {
        return Commands.runOnce(() -> setTargetVelocity(FlywheelConstants.kNeutralVelocity));
    }
  public void moveToNeutral(){
    setTargetVelocity(FlywheelConstants.kNeutralVelocity);
  }

   //****************************** MANUAL METHODS ******************************//
   public void setPercentageTop(double per){
    topMotor.setSpeed(per);
  }

    public void setPercentageBottom(double per){
    bottomMotor.setSpeed(per);
  }

  public void setPercentageShuffleBoard(){
    double topSpeed = sbManualTopSpeedPercent.getDouble(0);
    double bottomSpeed = sbManualBottomSpeedPercent.getDouble(0);

    setPercentageTop(topSpeed);
    setPercentageBottom(bottomSpeed);
    setEnabled(false);
  }

  
  public void setVelocityShuffleBoard(){
    double targetVelocity = sbManualTargetVelocity.getDouble(0);
    setTargetVelocity(targetVelocity);
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
    
    topMotor.configurePIDF(kP, kI, kD, kIzone, kF, kMinOut, kMaxOutput);
    bottomMotor.configurePIDF(kP, kI, kD, kIzone, kF, kMinOut, kMaxOutput);
  }

  public void initShuffleboard(int level) {
    //Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
    if (level == 0)  {
        return;
    }
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    switch (level) {
        case 0:
          break;
        case 1:
   
          tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

        case 2:
      
          tab.addNumber("Current Velocity", () -> getVelocity());
          tab.addNumber("Current Error", () -> getCurrentError());

        case 3:
          tab.addNumber("Top percent (motor controller)", () -> getTopAppliedVoltage());
          tab.addNumber("Bottom percent (motor controller)", () -> getTopAppliedVoltage());
          tab.addNumber("Target Velocity", () -> getTargetVelocity());
          tab.addNumber("Current Top Velocity", () -> getTopVelocity());
          tab.addNumber("Current Bottom Velocity", () -> getBottomVelocity());
          tab.addBoolean("At Target", () -> hasReachedVelocity(getTargetVelocity()));
    }
  }



}
