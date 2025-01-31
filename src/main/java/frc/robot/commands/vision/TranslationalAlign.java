// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TranslationalAlign extends Command {
  SwerveSubsystem m_SwerveSubsystem;
  PIDController xPID;
  PIDController yPID;
  NetworkTable limelight;
  double targetAngleThreshold = 0.1;

  /** Creates a new TranslationalAlign. */
  public TranslationalAlign(SwerveSubsystem m_SwerveSubsystem, NetworkTable limelight) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    xPID = new PIDController(0.1, 0.0, 0.1);
    yPID = new PIDController(0.1, 0.0, 0.1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = limelight.getEntry("tx").getDouble(0.0);
    double ty = limelight.getEntry("ty").getDouble(0.0);

    double driveX = xPID.calculate(tx, 0);
    double driveY = yPID.calculate(ty, 0);

    m_SwerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, m_SwerveSubsystem.getHeading(),  Rotation2d.fromDegrees(m_SwerveSubsystem.getHeading())), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getEntry("tx").getDouble(0.0)) < targetAngleThreshold;
  }
}
