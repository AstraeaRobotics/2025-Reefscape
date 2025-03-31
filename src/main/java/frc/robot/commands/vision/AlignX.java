// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SwerveUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignX extends Command {
  /** Creates a new AlignLeft. */
  SwerveSubsystem m_SwerveSubsystem;

  double tx;
  PIDController offsetController;
  double setpoint;

  public AlignX(SwerveSubsystem m_SwerveSubsystem, double setpoint) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.setpoint = setpoint;

    addRequirements(m_SwerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    offsetController = new PIDController(0.006, 0, 0);
    // offsetController.setSetpoint(setpoint);
    // offsetController.setTolerance(0.01);

    tx = LimelightHelpers.getTX("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = LimelightHelpers.getTX("limelight");
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("limelight pid output", offsetController.calculate(tx));

    m_SwerveSubsystem.drive(SwerveUtil.driveInputToChassisSpeeds(-offsetController.calculate(tx, setpoint), 0, 0, 0), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(SwerveUtil.driveInputToChassisSpeeds(0, 0, 0, 0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return offsetController.atSetpoint();
    return false;
  }
}
