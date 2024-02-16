// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Indexer extends SubsystemBase {

  private final CANSparkMax topBelt;
  private final CANSparkMax bottomBelt;

  double targetPosition;

  private boolean troubleshooting;

  /** Creates a new Indexer. */
  public Indexer() {

    this.targetPosition = 0;

    troubleshooting = true;

    topBelt = new CANSparkMax(FlywheelConstants.kTopShooterID, MotorType.kBrushless);
    topBelt.restoreFactoryDefaults();
    topBelt.setSmartCurrentLimit(50);
    topBelt.setIdleMode(IdleMode.kBrake);
    topBelt.setOpenLoopRampRate(0.2);
    topBelt.setInverted(false);

    topBelt.getPIDController().setP(0);
    topBelt.getPIDController().setI(0);
    topBelt.getPIDController().setD(0);
    topBelt.getPIDController().setFF(0);

    bottomBelt = new CANSparkMax(FlywheelConstants.kTopShooterID, MotorType.kBrushless);
    bottomBelt.restoreFactoryDefaults();
    bottomBelt.setSmartCurrentLimit(50);
    bottomBelt.setIdleMode(IdleMode.kBrake);
    bottomBelt.setOpenLoopRampRate(0.2);
    bottomBelt.setInverted(false);

    bottomBelt.getPIDController().setP(0);
    bottomBelt.getPIDController().setI(0);
    bottomBelt.getPIDController().setD(0);
    bottomBelt.getPIDController().setFF(0);
    
  }

    
  public void configurePIDF(double kP, double kI, double kD, double kF){
       
    topBelt.getPIDController().setP(kP);
    topBelt.getPIDController().setI(kI);
    topBelt.getPIDController().setD(kD);
    topBelt.getPIDController().setFF(kF);

    bottomBelt.getPIDController().setP(kP);
    bottomBelt.getPIDController().setI(kI);
    bottomBelt.getPIDController().setD(kD);
    bottomBelt.getPIDController().setFF(kF);
  }



  public void setReferenceVelocity(double velocity){
    topBelt.getPIDController().setReference(velocity, ControlType.kVelocity);
    bottomBelt.getPIDController().setReference(velocity, ControlType.kVelocity);

  }

  public void setReferencePosition(){
    topBelt.getPIDController().setReference(targetPosition, ControlType.kPosition);
    bottomBelt.getPIDController().setReference(targetPosition, ControlType.kPosition);

  }


  public double getTopAppliedVoltage(){
    return topBelt.getOutputCurrent();
  }

   public double getTopTemperature(){
    return topBelt.getMotorTemperature();
  }

    public double getBottomAppliedVoltage(){
    return bottomBelt.getOutputCurrent();
  }

   public double getBottomTemperature(){
    return bottomBelt.getMotorTemperature();
  }

  public void setVoltage(double voltage){
    topBelt.set(voltage);
    bottomBelt.set(voltage);
  }

  public double getTopVelocityRPM(){
    return topBelt.getEncoder().getVelocity();
  }

  public double getTopPosition(){
    return topBelt.getEncoder().getPosition();
  }

    public double getBottomVelocityRPM(){
    return bottomBelt.getEncoder().getVelocity();
  }

  public double getBottomPosition(){
    return bottomBelt.getEncoder().getPosition();
  }

  public void setPosition(double pos){
    topBelt.getEncoder().setPosition(pos);
    bottomBelt.getEncoder().setPosition(pos);
  }

  public void setTargetPosition(double pos){
    this.targetPosition = pos;
  }

  public void setTargetPositionIncrement(double increment){
    this.targetPosition += increment;
  }

  public void stop(){
    topBelt.set(0);
    bottomBelt.set(0);
    
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
