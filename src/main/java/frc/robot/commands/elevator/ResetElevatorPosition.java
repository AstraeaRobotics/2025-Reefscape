// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetElevatorPosition extends InstantCommand {
  ElevatorSubsystem m_ElevatorSubsystem;
  public ResetElevatorPosition(ElevatorSubsystem m_ElevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = m_ElevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.resetPosition();
  }
}
