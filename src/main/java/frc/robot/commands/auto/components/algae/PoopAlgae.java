// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components.algae;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.subsystems.AlgaeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoopAlgae extends SequentialCommandGroup {
  /** Creates a new PoopAlgae. */
  AlgaeSubsystem m_AlgaeSubsystem;

  public PoopAlgae(AlgaeSubsystem m_AlgaeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(1), new IntakeAlgae(m_AlgaeSubsystem, -5)));
  }
}
