// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.subsystems.AlgaeIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAlgaeState extends InstantCommand {

  private final AlgaeIntake m_algae;
  private final AlgaeStates m_newState;

  public SetAlgaeState(AlgaeIntake intake, AlgaeStates state) {
    m_algae = intake;
    m_newState = state;

    addRequirements(m_algae);
  }

  @Override
  public void initialize() {
    m_algae.setState(m_newState);
  }
}
