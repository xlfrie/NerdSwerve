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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SubSystemConfigs;
import frc.robot.utils.CanController;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */

    private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake Troubleshooting");
    private GenericEntry sbManualSpeedPercent = intakeTab.add("Speed Percent In", 0).withPosition(0, 0).getEntry();
    private GenericEntry sbManualTargetVelocity = intakeTab.add("Target Velocity In", 0).withPosition(1, 0).getEntry();

    private final CanController intakeMotor;
    private boolean enabled = SubSystemConfigs.kEnableIntake;
    private boolean troubleshooting = true;
    private double targetRPM;

    private boolean aboveIntakeVelocity;

    public Intake() {
        intakeMotor = new CanController(IntakeConstants.kIntakeID);
        intakeMotor.configureMotor(IntakeConstants.kRotorToSensorRatio, IntakeConstants.kOpenLoopRampRate,
                IntakeConstants.kControlFramePeriod, IntakeConstants.kEncoderControlFramePeriod,
                IntakeConstants.kInverted, IntakeConstants.kIdleBrake);
        intakeMotor.configurePIDF(0.0001, 0, 0, 0, 0, 0, 1);
        aboveIntakeVelocity = false;
    }

    public void stop() {
        intakeMotor.stop();
    }

    public void setAboveIntake(boolean bol) {
        this.aboveIntakeVelocity = bol;
    }

    public boolean getAboveIntake() {
        return aboveIntakeVelocity;
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public void controlledIntake() {
        intakeMotor.setVelocityControl(2400);

    }

    public void setTargetVelocityShuffleboard() {
        double vel = sbManualTargetVelocity.getDouble(0);
        setTargetVelocity(vel);
        setEnabled(true);

    }

    public void setEnabled(boolean bol) {
        this.enabled = bol;
    }

    public void enableIntake() {
        this.enabled = true;
    }

    public Command enableIntakeCommand() {
        return Commands.runOnce(() -> this.enableIntake());
    }

    public double getTargetVelocity() {
        return targetRPM;
    }

    public boolean getEnabled() {
        return this.enabled;
    }

    public void setTargetVelocity(double vel) {
        this.targetRPM = vel;
    }

    public double getAppliedVoltage() {
        return intakeMotor.getMotorVoltage();
    }

    public void setPercentage(double per) {
        intakeMotor.setSpeed(per);
    }

    public void setPercentageShuffleBoard() {
        double speed = sbManualSpeedPercent.getDouble(0);
        setPercentage(speed);
    }

    public void setVelocityControl(double vel) {
        intakeMotor.setVelocityControl(vel);
    }

    public void initShuffleboard(int level) {
        // Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
        if (level == 0) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");

        switch (level) {
            case 0:
                break;
            case 1:
                tab.addString("Current Command",
                        () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
            case 2:
                tab.addBoolean("Enable Intake PID", () -> getEnabled());
                tab.addNumber("Target Velocity", () -> getTargetVelocity());

            case 3:
                tab.addNumber("Current Voltage", () -> getAppliedVoltage());
                tab.addNumber("Current Velocity", () -> getVelocity());
        }
    }

    @Override
    public void periodic() {

        if (enabled) {
            setVelocityControl(2400);
            if (getVelocity() > 750) {
                setAboveIntake(true);
            }
            if ((getAboveIntake()) & (getVelocity() < 600)) {
                setAboveIntake(false);
                stop();
                setEnabled(false);
            }
        }

        // if (getEnabled()){
        // setVelocityControl(targetRPM);
        // }
        // else{
        // if (troubleshooting){

        // }
        // else{
        // stop();
        // }
        // }

        // This method will be called once per scheduler run
    }
}
