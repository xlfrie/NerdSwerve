// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.WrongMethodTypeException;

import javax.sound.sampled.ReverbType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax wristMotor;
  private final DutyCycleEncoder throughBore;

  //I think the wrist runnin
  private boolean enabled = true;
  private boolean smartMotionEnabled = true;
  private boolean troubleshooting = true;

  private double targetPosition = 0;

  public Wrist() {
    wristMotor = new CANSparkMax(0, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(50);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setOpenLoopRampRate(0.2);
    wristMotor.setInverted(false);

    throughBore = new DutyCycleEncoder(0);

    configurePIDF(0, 0, 0, 0);
    resetEncoder();
  
  }

  public void configurePIDF(double kP, double kI, double kD, double kF){
       
    wristMotor.getPIDController().setP(kP);
    wristMotor.getPIDController().setI(kI);
    wristMotor.getPIDController().setD(kD);
    wristMotor.getPIDController().setIZone(0);
    wristMotor.getPIDController().setFF(kF);
    wristMotor.getPIDController().setOutputRange(0, 0.9);

    int smartMotionSlot = 0;
    wristMotor.getPIDController().setSmartMotionMaxVelocity(0, smartMotionSlot);;
    wristMotor.getPIDController().setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    wristMotor.getPIDController().setSmartMotionMaxAccel(0, smartMotionSlot);
    wristMotor.getPIDController().setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

  }

  public void resetEncoder(){
    throughBore.setPositionOffset(0);
    
    wristMotor.getEncoder().setPosition(0);
    wristMotor.getEncoder().setPositionConversionFactor(0);
  }

  public void zeroAbsoluteEncoder(){
    throughBore.setPositionOffset(throughBore.getAbsolutePosition());
  }

  public void stop(){
    wristMotor.set(0);
  }


  @Override
  public void periodic() {
    
    
    if (enabled) {
      if (smartMotionEnabled){
        wristMotor.getPIDController().setReference(targetPosition, ControlType.kSmartMotion);
      }
      else{
        wristMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
      }
    } 
  else {
  //    stop();

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
    return wristMotor.getEncoder().getPosition();
  }

  public double getAbsolutePosition() {
    if (true) {
        return throughBore.getPositionOffset() - throughBore.getAbsolutePosition();
    }
    return throughBore.getAbsolutePosition() - throughBore.getPositionOffset();
  }

  public boolean hasReachedPosition(double position){
    if (Math.abs(getAbsolutePosition()-position)<0){
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

      //****************************** POSITION METHODS ******************************//

    
  public Command moveToNeutralCommand() {
        return Commands.runOnce(() -> setTargetPosition(0));
    }
  public void moveToNeutral(){
    setTargetPosition(0);
  }
  
      //****************************** MANUAL METHODS ******************************//
  public void setPercentage(double per){
    wristMotor.set(per);
  }

}
