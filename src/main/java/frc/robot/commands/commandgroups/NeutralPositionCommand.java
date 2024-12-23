// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakePercentCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NeutralPositionCommand extends ParallelCommandGroup {
    /** Creates a new NeutralPositionCommand. */
    public NeutralPositionCommand(Arm arm, Wrist wrist, Intake intake) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(arm.moveToNeutralCommand(), wrist.moveToNeutralCommand(), new IntakePercentCommand(intake, 0));
    }
}
