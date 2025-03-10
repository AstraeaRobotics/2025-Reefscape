// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components.algae;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.commands.algae.SetAlgaeState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RemoveAlgaeL2 extends SequentialCommandGroup {
  /** Creates a new RemoveAlgaeL2. */
  public RemoveAlgaeL2(AlgaeSubsystem m_AlgaeSubsystem, ElevatorSubsystem m_ElevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(1), new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kAL2), new ParallelDeadlineGroup(new WaitCommand(0.5), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kL2))), new ParallelDeadlineGroup(new WaitCommand(1), new IntakeAlgae(m_AlgaeSubsystem, 5)));
  }
}
