// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.WrongMethodTypeException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax wristMotor;
  private final DutyCycleEncoder throughBore;

  //I think the wrist runnin
  private boolean enabled = true;

  public Wrist() {
    wristMotor = new CANSparkMax(0, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(50);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setOpenLoopRampRate(0.2);
    wristMotor.setInverted(false);
  
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
