// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationalAlign extends Command {
  SwerveSubsystem m_SwerveSubsystem;
  LimelightUtil limelight;
  PIDController turnPID;
  double turnKP;
  double turnKI; 
  double turnKD;

  public RotationalAlign(SwerveSubsystem m_SwerveSubsystem) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double rotationSpeed = turnPID.calculate(LimelightUtil.getTx(), 0);
      m_SwerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, rotationSpeed, Rotation2d.fromDegrees(m_SwerveSubsystem.getHeading())), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
