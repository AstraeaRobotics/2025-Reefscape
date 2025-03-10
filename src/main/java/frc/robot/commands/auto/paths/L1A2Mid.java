// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.auto.components.algae.PoopAlgae;
import frc.robot.commands.auto.components.algae.RemoveAlgaeL2;
import frc.robot.commands.auto.components.coral.ScoreL1;
import frc.robot.commands.auto.components.drivebase.DriveToDistance;
import frc.robot.commands.auto.components.drivebase.TurnToAngle;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1A2Mid extends SequentialCommandGroup {
  /** Creates a new L1A2Mid. */
  ElevatorSubsystem m_ElevatorSubsystem;
  AlgaeSubsystem m_AlgaeSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  CoralSubsystem m_CoralSubsystem;

  public L1A2Mid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToDistance(m_SwerveSubsystem, 0, 2.24), new ScoreL1(m_ElevatorSubsystem, m_CoralSubsystem), new RemoveAlgaeL2(m_AlgaeSubsystem, m_ElevatorSubsystem), new DriveToDistance(m_SwerveSubsystem, 0, -0.3), new TurnToAngle(m_SwerveSubsystem, 90), new PoopAlgae(m_AlgaeSubsystem), new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest));
  }
}
