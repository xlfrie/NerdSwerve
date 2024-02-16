// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleToIntFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  
  private final CANSparkMax intake;
  /** Creates a new IntakeRoller. */

   ShuffleboardTab sbIntakeTab = Shuffleboard.getTab("Intake Roller");
   GenericEntry sbCurrentRPM = sbIntakeTab.add("Current RPM", 0).withPosition(0, 0).getEntry();
   GenericEntry sbTargettedRPM = sbIntakeTab.add("Targetted RPM", 0).withPosition(1, 0).getEntry();
   GenericEntry sbCurrentVoltage = sbIntakeTab.add("Current Voltage", 0).withPosition(2, 0).getEntry();
   GenericEntry sbCurrentTemperature = sbIntakeTab.add("Current Temperature", 0).withPosition(3, 0).getEntry();


   GenericEntry sbKp = sbIntakeTab.add("kP", 0).withPosition(0, 2).getEntry();
   GenericEntry sbKi =sbIntakeTab.add("kI", 0).withPosition(1, 2).getEntry();
   GenericEntry sbKd = sbIntakeTab.add("kD", 0).withPosition(2, 2).getEntry();
   GenericEntry sbKf =sbIntakeTab.add("kF", 0).withPosition(3, 2).getEntry();
   GenericEntry sbTunningPID = sbIntakeTab.add("Tune PID", 0).withPosition(4, 2).getEntry();

  private double targetRPM;

  private boolean troubleshooting;

  public IntakeRoller() {
    targetRPM = 0;
    troubleshooting = true;

    intake = new CANSparkMax(0, MotorType.kBrushless);

    intake.restoreFactoryDefaults();
    intake.setInverted(false);
    intake.setSmartCurrentLimit(50);
    intake.setIdleMode(IdleMode.kBrake);
    intake.setOpenLoopRampRate(0.2);
    
    intake.getPIDController().setP(0);
    intake.getPIDController().setI(0);
    intake.getPIDController().setD(0);
    intake.getPIDController().setFF(0);


  }

  public void configurePIDF(double kP, double kI, double kD, double kF){
       
    intake.getPIDController().setP(kP);
    intake.getPIDController().setI(kI);
    intake.getPIDController().setD(kD);
    intake.getPIDController().setFF(kF);
    
  }

  public void stop(){
    intake.set(0);
  }

  public void setVelocity(double velocity){
    intake.getPIDController().setReference(velocity, ControlType.kVelocity);
  }


  public double getAppliedVoltage(){
    return intake.getOutputCurrent();
  }

   public double getTemperature(){
    return intake.getMotorTemperature();
  }


  public void setVoltage(double voltage){
    intake.set(voltage);
  }

  public double getVelocityRPM(){
    return intake.getEncoder().getVelocity();
  }

  public double getPosition(){
    return intake.getEncoder().getPosition();
  }

  public void setPosition(double pos){
    intake.getEncoder().setPosition(pos);
  }

  public void setTargetRPM(double vel){
    targetRPM = vel;
  }

  public void updateShuffleboard(){

    sbCurrentRPM.setDouble(getVelocityRPM());
    sbCurrentVoltage.setDouble(getAppliedVoltage());
    sbCurrentTemperature.setDouble(getTemperature());

    

  }
  @Override
  public void periodic() {
      if (troubleshooting) {
        if (sbTunningPID.getDouble(0)==1){
          
          double kP = sbKp.getDouble(0);
          double kI = sbKi.getDouble(0);
          double kD = sbKd.getDouble(0);
          double kF = sbKf.getDouble(0);
          
          configurePIDF(kP, kI, kD, kF);


        }

      }
    // This method will be called once per scheduler run
  }
}


