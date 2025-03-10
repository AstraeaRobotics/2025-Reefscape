// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.Coral.ExtakeL1;
import frc.robot.commands.Coral.IntakeCoral;
import frc.robot.commands.Coral.SetCoralState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL1 extends SequentialCommandGroup {
  /** Creates a new ScoreL1. */
  public ScoreL1(ElevatorSubsystem m_ElevatorSubsystem, CoralSubsystem m_CoralSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new ParallelDeadlineGroup(new WaitCommand(1.5), new SetCoralState(m_CoralSubsystem, CoralStates.kL1), new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1)), new ParallelDeadlineGroup(new WaitCommand(1), new ExtakeL1(m_CoralSubsystem)));
  }
}
