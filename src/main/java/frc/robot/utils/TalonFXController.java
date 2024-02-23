// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.opencv.core.Point;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class TalonFXController {
    
    private TalonFX motor;
    private TalonFXConfigurator motorConfigurator;
    private  NeutralOut brakeRequest;
    private MotionMagicVoltage motionMagicRequest;
    private VelocityVoltage motorVelocityRequest;
    private PositionVoltage motorPositionRequest;
    private TalonFXConfiguration motorConfigs;

    private DutyCycleOut motorRequest;
    private Slot0Configs motorPIDConfigs;

    public TalonFXController(int motorID){
        this.motor = new TalonFX(motorID);
        this.motorConfigurator = this.motor.getConfigurator();
        this.motorRequest = new DutyCycleOut(0);

        this.motorVelocityRequest = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        this.motorVelocityRequest.Slot = 0;

        this.motionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
        this.motionMagicRequest.Slot = 0;

        this.motorPositionRequest = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
        this.motorPositionRequest.Slot = 0;

        this.brakeRequest = new NeutralOut();
        



        

    }


    public void configureMotor(double rotorToSensorRatio, double sensorToMechanismRatio,double dutyCycleNeutralDeadband, boolean invertClockwise, boolean idleBrake ) {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigurator.refresh(motorConfigs);
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Feedback.RotorToSensorRatio = rotorToSensorRatio;
        motorConfigs.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

        if (invertClockwise){
        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        else{
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        }

        motorConfigs.Voltage.PeakForwardVoltage = 11.5;
        motorConfigs.Voltage.PeakReverseVoltage = -11.5;
        
        if (idleBrake){
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        else{
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = dutyCycleNeutralDeadband;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        motorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        motorConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode motorStatus = motorConfigurator.apply(motorConfigs);
        if (!motorStatus.isOK()){
            DriverStation.reportError("Could not apply  configs, error code:"+ motorConfigs.toString(), new Error().getStackTrace());
        }
    }

    public void configurePIDF(double kP, double kI, double kD, double kV, double kS, double kA, double kG, double cruiseVel, double accel){
        Slot0Configs motorPIDConfigs = new Slot0Configs();
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigurator.refresh(motorPIDConfigs);
        motorConfigs.Slot0.kP = kP;
        motorConfigs.Slot0.kI = kI;
        motorConfigs.Slot0.kD = kD;
        motorConfigs.Slot0.kV = kV;
        motorConfigs.Slot0.kS = kS;
        motorConfigs.Slot0.kA = kA;
        motorConfigs.Slot0.kG = kG;
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = cruiseVel;
        motorConfigs.MotionMagic.MotionMagicAcceleration   = accel;
        StatusCode configStatusCode = motorConfigurator.apply(motorConfigs);
        if (!configStatusCode.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:"+ configStatusCode.toString(), new Error().getStackTrace());
        }

        
    }

    //Encoder and logging stuffs
    public double getPosition(){
        return motor.getPosition().getValue();
    }

    public double getVelocity(){
        return motor.getVelocity().getValue();
    }
 
    public void setPosition(double pos){
        motor.setPosition(pos);
    }

    public double getMotorVoltage(){
        return motor.getMotorVoltage().getValue();
    }

    //Set Speed Stuffs
    public void stop(){
        motor.setControl(brakeRequest);
    }
    public void setSpeed(double percent){
        motorRequest.Output = percent;
        motor.setControl(motorRequest);

        //motor.set(percent);
    }  

    //PID Stuffs
    public void setVelocityControl(double velocity){
        motorVelocityRequest.Velocity = velocity;
        motor.setControl(motorVelocityRequest.withVelocity(velocity));
    }

    public void setPositionControl(double position){
        motorPositionRequest.Position = position;
        motor.setControl(motorPositionRequest.withPosition(position));
    }

    public void setSmartMotionPositionControl(double position){
        motionMagicRequest.Position = position;
        motor.setControl(motionMagicRequest.withPosition(position));
    }

    public Slot0Configs getMotorConfig(){
        return motorPIDConfigs;
    }

      public MotionMagicConfigs getMagicMotionMotorConfig(){
        return motorConfigs.MotionMagic;
    }

    public void initlializeShuffleboard(){

    }




}
