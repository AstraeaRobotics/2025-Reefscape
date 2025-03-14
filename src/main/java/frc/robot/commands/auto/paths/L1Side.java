// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.auto.components.drivebase.TurnToAngle;
import frc.robot.commands.auto.components.drivebase.TurnWheelsAndDrive;
import frc.robot.commands.coral.ExtakeL1;
import frc.robot.commands.coral.IntakeCoral;
import frc.robot.commands.coral.SetCoralState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1Side extends SequentialCommandGroup {
  /** Creates a new L2Side. */
  public L1Side(SwerveSubsystem m_SwerveSubsystem, CoralSubsystem m_CoralSubsystem, ElevatorSubsystem m_ElevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(2.7), new TurnWheelsAndDrive(m_SwerveSubsystem, 0, 2.75, 0)), new TurnToAngle(m_SwerveSubsystem, 300), new ParallelDeadlineGroup(new WaitCommand(2.7), new TurnWheelsAndDrive(m_SwerveSubsystem, -1.0, 0, 300)), new ParallelDeadlineGroup(new WaitCommand(1.5), new SetCoralState(m_CoralSubsystem, CoralStates.kL1), new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1)), new ParallelDeadlineGroup(new WaitCommand(1), new ExtakeL1(m_CoralSubsystem)), new ParallelDeadlineGroup(new WaitCommand(1.5), new SetCoralState(m_CoralSubsystem, CoralStates.kRest), new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest)));
  }
}
