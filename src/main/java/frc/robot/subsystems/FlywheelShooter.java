// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelShooter extends SubsystemBase {

   ShuffleboardTab sbShooterTab = Shuffleboard.getTab("Flywheel Shooter");
   GenericEntry sbBottomCurrentRPM = sbShooterTab.add("Bottom RPM", 0).withPosition(0, 0).getEntry();
   GenericEntry sbTopCurrentRPM = sbShooterTab.add("Top RPM", 0).withPosition(1, 0).getEntry();
   GenericEntry sbTopManualRPM = sbShooterTab.add("Top Manual RPM", 0).withPosition(0, 1).getEntry();
   GenericEntry sbBottomManualRPM = sbShooterTab.add("Bottom Manual RPM", 0).withPosition(1, 1).getEntry();
   GenericEntry sbTopManualPercent = sbShooterTab.add("Top Manual Percent", 0).withPosition(2, 1).getEntry();
   GenericEntry sbBottomManualPercent = sbShooterTab.add("Bottom Manual Percent", 0).withPosition(3, 1).getEntry();
   
   GenericEntry sbKp = sbShooterTab.add("kP", 0).withPosition(0, 2).getEntry();
   GenericEntry sbKi = sbShooterTab.add("kI", 0).withPosition(1, 2).getEntry();
   GenericEntry sbKd = sbShooterTab.add("kD", 0).withPosition(2, 2).getEntry();
   GenericEntry sbKf = sbShooterTab.add("kF", 0).withPosition(3, 2).getEntry();
   GenericEntry sbTunningPID = sbShooterTab.add("Tune PID", 0).withPosition(4, 2).getEntry();



  CANSparkMax topFlywheelMotor;
  CANSparkMax bottomFlywheelMotor;

  

  /** Creates a new FlywheelShooter. */
  public FlywheelShooter() {
    topFlywheelMotor = new CANSparkMax(FlywheelConstants.kTopShooterID, MotorType.kBrushless);
    topFlywheelMotor.restoreFactoryDefaults();
    topFlywheelMotor.setSmartCurrentLimit(50);
    topFlywheelMotor.setIdleMode(IdleMode.kBrake);
    topFlywheelMotor.setOpenLoopRampRate(0.2);
    topFlywheelMotor.setInverted(false);
    
    topFlywheelMotor.getPIDController().setP(FlywheelConstants.kPShooter);
    topFlywheelMotor.getPIDController().setI(FlywheelConstants.kIShooter);
    topFlywheelMotor.getPIDController().setD(FlywheelConstants.kDShooter);
    topFlywheelMotor.getPIDController().setFF(FlywheelConstants.kFShooter);
    
    bottomFlywheelMotor = new CANSparkMax(FlywheelConstants.kBottomShooterID, MotorType.kBrushless);
    bottomFlywheelMotor.restoreFactoryDefaults();
    bottomFlywheelMotor.setSmartCurrentLimit(50);
    bottomFlywheelMotor.setIdleMode(IdleMode.kBrake);
    bottomFlywheelMotor.setOpenLoopRampRate(0.2);
    bottomFlywheelMotor.setInverted(false);
    
    bottomFlywheelMotor.getPIDController().setP(FlywheelConstants.kPShooter);
    bottomFlywheelMotor.getPIDController().setI(FlywheelConstants.kIShooter);
    bottomFlywheelMotor.getPIDController().setD(FlywheelConstants.kDShooter);
    bottomFlywheelMotor.getPIDController().setFF(FlywheelConstants.kFShooter);
    
    //Will do smart motion in the future

  }

  public void configurePIDF(double kP, double kI, double kD, double kF){
       
    bottomFlywheelMotor.getPIDController().setP(kP);
    bottomFlywheelMotor.getPIDController().setI(kI);
    bottomFlywheelMotor.getPIDController().setD(kD);
    bottomFlywheelMotor.getPIDController().setFF(kF);

    topFlywheelMotor.getPIDController().setP(kP);
    topFlywheelMotor.getPIDController().setI(kI);
    topFlywheelMotor.getPIDController().setD(kD);
    topFlywheelMotor.getPIDController().setFF(kF);

  }

  public void stop(){
    topFlywheelMotor.set(0);
    bottomFlywheelMotor.set(0);
    
  }

  public void speedPercent(double topPercent, double bottomPercent, boolean manual){
    if (manual){
      double topManualPercent = sbTopManualPercent.getDouble(0);
      double bottomManualPercent = sbBottomManualPercent.getDouble(0);
      topFlywheelMotor.set(topManualPercent);
      bottomFlywheelMotor.set(bottomManualPercent);
    }
  
    else{
    topFlywheelMotor.set(topPercent);
    bottomFlywheelMotor.set(bottomPercent);
    }
  }


  public void speedRPM(double topRPM, double bottomRPM, boolean manual){
    if (manual){
      double topManualRPM = sbTopManualRPM.getDouble(0);
      double bottomManualRPM = sbBottomManualRPM.getDouble(0);

      topFlywheelMotor.getPIDController().setReference(topManualRPM, ControlType.kVelocity);
      bottomFlywheelMotor.getPIDController().setReference(bottomManualRPM, ControlType.kVelocity);

    }
    else{      
      topFlywheelMotor.getPIDController().setReference(topRPM, ControlType.kVelocity);
      bottomFlywheelMotor.getPIDController().setReference(bottomRPM, ControlType.kVelocity);

    }
  }

  public void updateShuffleboard(){
    sbTopCurrentRPM.setDouble(topFlywheelMotor.getEncoder().getVelocity());
    sbBottomCurrentRPM.setDouble(bottomFlywheelMotor.getEncoder().getVelocity());
  }



  @Override
  public void periodic() {

    if (sbTunningPID.getDouble(0)==1){
      double kP = sbKp.getDouble(0);
      double kI = sbKi.getDouble(0);
      double kD = sbKd.getDouble(0);
      double kF = sbKf.getDouble(0);

      configurePIDF(kP, kI, kD, kF);

    }

    updateShuffleboard();
    // This method will be called once per scheduler run
  }


}
