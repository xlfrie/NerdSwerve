// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheelshooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooter;

public class FlywheelRPM extends Command {
  /** Creates a new FlywheelRPM. */

  FlywheelShooter flywheelShooter;
  boolean manual;
  public FlywheelRPM(FlywheelShooter fly, boolean man) {
    this.manual = man;
    this.flywheelShooter = fly;
    addRequirements(fly);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manual){
      flywheelShooter.speedRPM(0, 0, manual);
    }
    else{
      flywheelShooter.speedRPM(0.5, 0.8, manual);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheelShooter.speedRPM(0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
