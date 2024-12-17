// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelShooter;

public class shooterPercentCommand extends Command {
    FlyWheelShooter shooter;
    double speed;

    /** Creates a new shooterPercentCommand. */
    public shooterPercentCommand(FlyWheelShooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setPercentageTop(speed);
        shooter.setPercentageBottom(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
