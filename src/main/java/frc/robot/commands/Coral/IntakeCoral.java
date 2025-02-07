// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

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
<<<<<<< Updated upstream
    m_CoralSubsystem.setLeftCoralIntakeMotor(speed);
    m_CoralSubsystem.setRightCoralIntakeMotor(speed);
=======
<<<<<<< HEAD
    m_CoralSubsystem.setcoralIntakeVoltage(voltage);
=======
    m_CoralSubsystem.setLeftCoralIntakeMotor(speed);
    m_CoralSubsystem.setRightCoralIntakeMotor(speed);
>>>>>>> 2d602620bfe5a00207fd21a4f919e8040b28a8e2
>>>>>>> Stashed changes
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< Updated upstream
    m_CoralSubsystem.setLeftCoralIntakeMotor(0);
    m_CoralSubsystem.setRightCoralIntakeMotor(0);
=======
<<<<<<< HEAD
    m_CoralSubsystem.setcoralIntakeVoltage(0);
=======
    m_CoralSubsystem.setLeftCoralIntakeMotor(0);
    m_CoralSubsystem.setRightCoralIntakeMotor(0);
>>>>>>> 2d602620bfe5a00207fd21a4f919e8040b28a8e2
>>>>>>> Stashed changes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
