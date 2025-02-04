// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeIntakeVoltageManually extends Command {
  /** Creates a new SetAlgaeIntakeVoltageManual. */

  private final AlgaeIntake m_algae;
  private final double m_volts;

  public SetAlgaeIntakeVoltageManually(AlgaeIntake algae, double voltage) {
    m_algae = algae;
    m_volts = voltage;
    
    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_algae.setIntakeVoltageManual(m_volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algae.setIntakeVoltageManual(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
