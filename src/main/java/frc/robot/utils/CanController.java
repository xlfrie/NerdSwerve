// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.fasterxml.jackson.databind.BeanProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */

public class CanController {

      

      private CANSparkMax motor;
      
      public CanController(int motorid){
            motor = new CANSparkMax(motorid, MotorType.kBrushless);


      }

      public void configureMotor(double rotorToSensorRatio, double openLoopRampRate, int controlFramePeriod, int encoderControlFramePeriod,boolean inverted,boolean idleBrake){
            motor.restoreFactoryDefaults();
            motor.setSmartCurrentLimit(50);

            if (idleBrake){
            motor.setIdleMode(IdleMode.kBrake);
            }
            else{
            motor.setIdleMode(IdleMode.kCoast);

            }
            motor.setOpenLoopRampRate(openLoopRampRate);
            motor.setInverted(inverted);
            motor.setControlFramePeriodMs(controlFramePeriod);
            
            //Encoder Stuffs
            motor.getEncoder().setMeasurementPeriod(encoderControlFramePeriod);
            motor.getEncoder().setPositionConversionFactor(rotorToSensorRatio);
            motor.getEncoder().setVelocityConversionFactor(rotorToSensorRatio);
      }

      public void configurePIDF(double kP, double kI, double kD, double kIz, double kF, double kMinOutput, double kMaxOutput){
       
            motor.getPIDController().setP(kP);
            motor.getPIDController().setI(kI);
            motor.getPIDController().setD(kD);
            motor.getPIDController().setIZone(0);
            motor.getPIDController().setFF(kF);
            motor.getPIDController().setOutputRange(0, 0.9);

          
          }
        
          public void configureSmartMotion(double kMaxVelocity, double kMinVelocity, double kMaxAccel, double kAllowedError ){
             int smartMotionSlot = 0;
            motor.getPIDController().setSmartMotionMaxVelocity(kMaxVelocity, smartMotionSlot);;
            motor.getPIDController().setSmartMotionMinOutputVelocity(kMinVelocity, smartMotionSlot);
            motor.getPIDController().setSmartMotionMaxAccel(kMaxAccel, smartMotionSlot);
            motor.getPIDController().setSmartMotionAllowedClosedLoopError(kAllowedError, smartMotionSlot);
            
          }

           //Encoder and logging stuffs
    public double getPosition(){
      return motor.getEncoder().getPosition();
  }

  public double getVelocity(){
      return motor.getEncoder().getVelocity();
  }

  public void setPosition(double pos){
      motor.getEncoder().setPosition(pos);
  }

  public double getMotorVoltage(){
      return motor.getAppliedOutput();
  }

  public double getCompensationVoltage(){
      return motor.getVoltageCompensationNominalVoltage();
  }

  //Set Speed Stuffs
      public void stop(){
            motor.set(0);
      }
  public void setSpeed(double percent){
      motor.set(percent);
  }  

  //PID Stuffs
  public void setVelocityControl(double velocity){
      motor.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  public void setPositionControl(double position){
      motor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setSmartMotionPositionControl(double position){
      motor.getPIDController().setReference(position, ControlType.kSmartMotion);
  }

  public SparkPIDController getPIDController(){
      return motor.getPIDController();
  }


  public void updateShuffleboard(){
      
}


}
