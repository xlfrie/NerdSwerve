// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelShooter;

public class ShooterEndAtPosition extends Command {
    /** Creates a new ShooterEndAtPosition. */
    FlyWheelShooter shooter;
    double targetPos;
    boolean isFinished;

    public ShooterEndAtPosition(FlyWheelShooter sh, double pos) {
        this.shooter = sh;
        this.targetPos = pos;
        isFinished = false;
        addRequirements(sh);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        shooter.resetEncoder(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(shooter.getPosition() - targetPos) < 200) {
            isFinished = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
