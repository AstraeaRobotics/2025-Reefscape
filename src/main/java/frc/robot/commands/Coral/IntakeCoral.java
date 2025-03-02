// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new IntakeCoral. */
  CoralSubsystem m_CoralSubsystem;
  double voltage;

  public IntakeCoral(CoralSubsystem m_CoralSubsystem, double voltage) {
    this.m_CoralSubsystem = m_CoralSubsystem;
    this.voltage = voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_CoralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralSubsystem.setIntakeVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralSubsystem.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
