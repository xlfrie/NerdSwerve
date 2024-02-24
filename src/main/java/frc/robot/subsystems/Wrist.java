// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.WrongMethodTypeException;

import javax.print.attribute.DocAttribute;
import javax.sound.sampled.ReverbType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.CanController;
import frc.robot.utils.TalonFXController;

public class Wrist extends SubsystemBase {

  //Troubleshooting - Set up tab

  private boolean troubleshooting = true;


  // private ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist Troubleshooting");
  // private GenericEntry sbManualSpeedPercent = wristTab.add("Speed Percent In", 0).withPosition(0, 0).getEntry();
  // private GenericEntry sbManualTargetPosition = wristTab.add("Target Position In", 0).withPosition(1, 0).getEntry();
  // private GenericEntry sbkPIn = wristTab.add("kP In ", 0).withPosition(0, 1).getEntry();
  // private GenericEntry sbkIIn = wristTab.add("kI In ", 0).withPosition(1, 1).getEntry();
  // private GenericEntry sbkDIn = wristTab.add("kD In ", 0).withPosition(2, 1).getEntry();
  // private GenericEntry sbkIZoneIn = wristTab.add("kIz In", 0).withPosition(3, 1).getEntry();
  // private GenericEntry sbkFIn = wristTab.add("kF In", 0).withPosition(4, 1).getEntry();
  // private GenericEntry sbkMinOut = wristTab.add("Min out", 0).withPosition(5, 1).getEntry();
  // private GenericEntry sbkMaxOut = wristTab.add("Max out", 0).withPosition(6, 1).getEntry();

  // private GenericEntry sbkMaxVelocity = wristTab.add("Max Velocity ", 0).withPosition(0, 2).getEntry();
  // private GenericEntry sbkMinVelocity = wristTab.add("Min Velocity", 0).withPosition(1, 2).getEntry();
  // private GenericEntry sbkMaxAccel = wristTab.add("Max Accel", 0).withPosition(2, 2).getEntry();
  // private GenericEntry sbkAllowedError = wristTab.add("Allowed Error", 0).withPosition(3, 2).getEntry();
     
 private ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist Troubleshooting");
    private GenericEntry sbManualSpeedPercent = wristTab.add("Speed Percent In", 0).withPosition(0, 0).getEntry();
    private GenericEntry sbManualTargetPosition = wristTab.add("Target Position In", 0).withPosition(1, 0).getEntry();
    private GenericEntry sbkPIn = wristTab.add("kP In ", 0).withPosition(0, 1).getEntry();
    private GenericEntry sbkIIn = wristTab.add("kI In ", 0).withPosition(1, 1).getEntry();
    private GenericEntry sbkDIn = wristTab.add("kD In ", 0).withPosition(2, 1).getEntry();
    private GenericEntry sbkVIn = wristTab.add("kV In", 0).withPosition(3, 1).getEntry();
    private GenericEntry sbkSIn = wristTab.add("kS In", 0).withPosition(4, 1).getEntry();
    private GenericEntry sbkAIn = wristTab.add("kA In", 0).withPosition(0, 2).getEntry();
    private GenericEntry sbkGIn = wristTab.add("kG In", 0).withPosition(1, 2).getEntry();
    private GenericEntry sbkCruiseVelocity = wristTab.add("Cruise Velocity", 0).withPosition(2, 2).getEntry();
    private GenericEntry sbkAcceleration = wristTab.add("Acceleration", 0).withPosition(3, 2).getEntry();
    private GenericEntry sbConfigurePID = wristTab.add("Configure PID", 0).withPosition(4, 2).getEntry();


  /** Creates a new Wrist. */
  private final TalonFXController wristMotor;
  private final Encoder throughBore;
  

  //I think the wrist runnin
  private boolean enabled = SubSystemConfigs.kEnableWrist;
  private boolean smartMotionEnabled = false;

  private double targetPosition = 0;
  private PIDController wristPIDController;


  public Wrist() {

    wristMotor = new TalonFXController(WristConstants.kWristMotorID);
    wristMotor.configureMotor(WristConstants.kRotorToSensorRatio, WristConstants.kSensorToMechanismRatio, WristConstants.kDutyCycleNeutralDeadband, WristConstants.kInvertClockwise, WristConstants.kIdleBrake);
    wristMotor.configurePIDF(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.kV, WristConstants.kS, WristConstants.kA, WristConstants.kG, WristConstants.kCruiseVel, WristConstants.kAccel);
    throughBore = new Encoder(9, 8);

    wristPIDController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
  
    resetEncoder();

  
  }

  
public boolean getEnabled(){
  return this.enabled;
}

  public void resetEncoder(){
    throughBore.reset();
    wristMotor.setPosition(0);
  }



  public void stop(){
    wristMotor.stop();
  }


  @Override
  public void periodic() {
    
    
    if (enabled) {
      if (smartMotionEnabled){
        wristMotor.setSmartMotionPositionControl(targetPosition);
      }
      else{
        wristMotor.setPositionControl(targetPosition);
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
    }
  }

    //****************************** STATE METHODS ******************************/
  public double getTargetPosition(){
    return targetPosition;
  }
  public void setTargetPosition(double pos){
    this.targetPosition = pos;

  }
  public double getPosition(){
    return wristMotor.getPosition();
  }

  public double getAbsolutePosition() {
    return throughBore.getRaw();
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

  public double getAppliedVoltage(){
    return wristMotor.getMotorVoltage();
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
        return Commands.runOnce(() -> setTargetPosition(WristConstants.kNeutralPosition));
    }
  public void moveToNeutral(){
    setTargetPosition(WristConstants.kNeutralPosition);
  }

  public void moveToGroundIntake(){
    setTargetPosition(WristConstants.kGroundIntakePosition);
    setEnabled(true);
  }
  public Command moveToGroundIntakeCommand() {
    return Commands.runOnce(() -> moveToGroundIntake());
}
  public void moveToShooterFeeding(){
    setTargetPosition(WristConstants.kShooterFeedingPosition);
    setEnabled(true);
  }

  public Command moveToShooterFeedingCommand(){
        return Commands.runOnce(() -> moveToShooterFeeding());
  }

  public void moveToAmpScoring(){
    setTargetPosition(WristConstants.kAmpScoringPosition);
    setEnabled(true);
  }
  public Command moveToAmpScoringCommand() {

    return Commands.runOnce(() -> moveToAmpScoring());
}
      //****************************** MANUAL METHODS ******************************//
  public void setPercentage(double per){
    wristMotor.setSpeed(per);
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
    // double kP = sbkPIn.getDouble(0);
    // double kI = sbkIIn.getDouble(0);
    // double kD = sbkDIn.getDouble(0);
    // double kIzone = sbkIZoneIn.getDouble(0);
    // double kF = sbkFIn.getDouble(0);
    // double kMinOut = sbkMinOut.getDouble(0);
    // double kMaxOutput = sbkMaxOut.getDouble(0);

    double kP = sbkPIn.getDouble(0);
    double kI = sbkIIn.getDouble(0);
    double kD = sbkDIn.getDouble(0);
    double kV = sbkVIn.getDouble(0);
    double kS = sbkSIn.getDouble(0);
    double kA =  sbkAIn.getDouble(0);
    double kG =  sbkGIn.getDouble(0);
    double kCruiseVelocity =  sbkCruiseVelocity.getDouble(0);
    double kAcceleration =  sbkAcceleration.getDouble(0);
    wristMotor.configurePIDF(kP, kI, kD, kV, kS, kA, kG, kCruiseVelocity, kAcceleration);
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
          tab.addNumber("Turn percent (motor controller)", () -> getAppliedVoltage());
          tab.addNumber("Target Position", () -> getTargetPosition());
          tab.addNumber("Encoder Position", () -> getPosition());
          tab.addBoolean("At Target", () -> hasReachedPosition(getTargetPosition()));
    }
  }

}
