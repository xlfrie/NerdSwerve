// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.commands.Shooter.ShooterEndAtPosition;
import frc.robot.commands.Shooter.shooterPercentCommand;
import frc.robot.commands.arm.ArmEndAtSetpoint;
import frc.robot.commands.intake.IntakePercentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FlyWheelShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterFeedingCommand extends SequentialCommandGroup {
  /** Creates a new ShooterFeedingCommand. */
  public ShooterFeedingCommand(Arm arm, Intake intake, Wrist wrist, FlyWheelShooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(arm.moveToShooterFeedingCommand(), wrist.moveToShooterFeedingCommand()), new ArmEndAtSetpoint(arm),  
    new ParallelCommandGroup(new IntakePercentCommand(intake, 0.15),new shooterPercentCommand(shooter, -0.15)));
  }
}

//,new ShooterEndAtPosition(shooter, -1500),new ParallelCommandGroup(new IntakePercentCommand(intake, 0),new shooterPercentCommand(shooter, 0)), new ParallelCommandGroup(arm.moveToNeutralCommand(), wrist.moveToNeutralCommand()));
