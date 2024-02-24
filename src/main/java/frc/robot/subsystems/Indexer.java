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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.CanController;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  private ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer Troubleshooting");
  private GenericEntry sbManualSpeedPercent = indexerTab.add("Speed Percent In", 0).withPosition(0, 0).getEntry();
    
  private GenericEntry sbkPIn = indexerTab.add("kP In ", 0).withPosition(0, 1).getEntry();
  private GenericEntry sbkIIn = indexerTab.add("kI In ", 0).withPosition(1, 1).getEntry();
  private GenericEntry sbkDIn = indexerTab.add("kD In ", 0).withPosition(2, 1).getEntry();
  private GenericEntry sbkIZoneIn = indexerTab.add("kIz In", 0).withPosition(3, 1).getEntry();
  private GenericEntry sbkFIn = indexerTab.add("kF In", 0).withPosition(4, 1).getEntry();
  private GenericEntry sbkMinOut = indexerTab.add("Min out", 0).withPosition(5, 1).getEntry();
  private GenericEntry sbkMaxOut = indexerTab.add("Max out", 0).withPosition(6, 1).getEntry();
  private GenericEntry sbConfigurePID = indexerTab.add("Configure PID", 0).withPosition(0, 2).getEntry();



  private final CanController topIndexerMotor;
  private final CanController bottomIndexerMotor;
  private boolean enabled = SubSystemConfigs.kEnableIndexer;
  private double targetVelocity = 0;

  private boolean troubleshooting = true;

  public Indexer() {

    topIndexerMotor = new CanController(IndexerConstants.kTopIndexerID);
    bottomIndexerMotor = new CanController(IndexerConstants.kBottomIndexerID);


    topIndexerMotor.configureMotor( IndexerConstants.kRotorToSensorRatio, IndexerConstants.kOpenLoopRampRate, IndexerConstants.kControlFramePeriod, IndexerConstants.kEncoderControlFramePeriod, IndexerConstants.kTopInverted, IndexerConstants.kIdleBrake);
    topIndexerMotor.configurePIDF(IndexerConstants.kP, IndexerConstants.kI, IndexerConstants.kD, IndexerConstants.kIz, IndexerConstants.kF, IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);

    bottomIndexerMotor.configureMotor( IndexerConstants.kRotorToSensorRatio, IndexerConstants.kOpenLoopRampRate, IndexerConstants.kControlFramePeriod, IndexerConstants.kEncoderControlFramePeriod, IndexerConstants.kBottomInverted, IndexerConstants.kIdleBrake);
    bottomIndexerMotor.configurePIDF(IndexerConstants.kP, IndexerConstants.kI, IndexerConstants.kD, IndexerConstants.kIz, IndexerConstants.kF, IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);

  }

  
  public void resetEncoder(double pos){
    topIndexerMotor.setPosition(pos);
    bottomIndexerMotor.setPosition(pos);
  }

  public double getTopVelocity(){
    return topIndexerMotor.getVelocity();
  }

    public double getBottomVelocity(){
    return bottomIndexerMotor.getVelocity();
  }

  public double getTopPosition(){
    return topIndexerMotor.getPosition();
  }


  public double getBottomPosition(){
    return bottomIndexerMotor.getPosition();
  }


  public void stop(){
    topIndexerMotor.stop();
    bottomIndexerMotor.stop();
  }

  public Command stopCommand() {
      return Commands.runOnce(() -> stop());
  }
  public double getAppliedVoltage(){
    return topIndexerMotor.getMotorVoltage();
  }

  public void setPercentage(double per){
    topIndexerMotor.setSpeed(per);
    bottomIndexerMotor.setSpeed(per);
  }

  public void setPercentageShuffleBoard(){
    double topSpeed = sbManualSpeedPercent.getDouble(0);

    setPercentage(topSpeed);

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
    
    topIndexerMotor.configurePIDF(kP, kI, kD, kIzone, kF, kMinOut, kMaxOutput);
  }

  public void setTargetVelocity(double pos){
    this.targetVelocity = pos;
  }

  public double gettargetVelocity(){
    return this.targetVelocity;
  }



  public void setVelocityControl(double vel){
    topIndexerMotor.setVelocityControl(vel);
    bottomIndexerMotor.setVelocityControl(vel);
  }

  public void setEnabled(boolean bol){
    this.enabled = bol;
  }

  public boolean getEnabled(){
    return enabled;
  }

  public void runPIDPosition(){
    topIndexerMotor.setPositionControl(targetVelocity);
    bottomIndexerMotor.setPositionControl(targetVelocity);
  }

  public boolean hasReachedPosition(double position){
    if (Math.abs(getTopPosition()-position)<IndexerConstants.kAllowedError){
      return true;
    }
    else{
      return false;
    }
  }


 //Setpoints

  public Command intakeOneNoteCommand() {
    return Commands.runOnce(() -> intakeOneNote());
}
  public void intakeOneNote(){
    setTargetVelocity(1000);
    setEnabled(true);
  
  }

  


  public void initShuffleboard(int level) {
    //Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
    if (level == 0)  {
        return;
    }
    ShuffleboardTab tab = Shuffleboard.getTab("Indexer");

    switch (level) {
        case 0:
          break;
        case 1:
   
          tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

        case 2:
          tab.addNumber("Current Top Position", () -> getTopPosition());
          tab.addNumber("Current Bottom Position", () -> getTopPosition());
          tab.addNumber("Target Position", () -> gettargetVelocity());

 

        case 3:
          tab.addNumber("Current Voltage", () -> getAppliedVoltage());
          tab.addNumber("Current Top Velocity", () -> getTopVelocity());
          tab.addNumber("Current Bottom Velocity", () -> getBottomVelocity());


    }
  }


  @Override
  public void periodic() {

    if (enabled){
      setVelocityControl(targetVelocity);    
    }
    else{
    if (troubleshooting){
      if (sbConfigurePID.getDouble(0)==1){
        updatePIDFValueShuffleBoard();
      }
    }
    else{
      stop();
    }
    // This method will be called once per scheduler run
  }
  }
}

