// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SimpleTestAuto extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double startTime;

  public SimpleTestAuto(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("=== SIMPLE TEST AUTO STARTED ===");
    startTime = System.currentTimeMillis() / 1000.0;
    m_swerveSubsystem.resetEncoders();
  }

  @Override
  public void execute() {
    // Drive forward at 1 m/s for 3 seconds
    ChassisSpeeds speeds = new ChassisSpeeds(0, 1.0, 0); // vx, vy, omega (robot-relative)
    m_swerveSubsystem.drive(speeds, false);
    
    System.out.println("Driving... Pose: " + m_swerveSubsystem.getPose().getX() + ", " + m_swerveSubsystem.getPose().getY());
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), false);
    System.out.println("=== SIMPLE TEST AUTO ENDED ===");
    System.out.println("Final Pose: " + m_swerveSubsystem.getPose());
  }

  @Override
  public boolean isFinished() {
    double elapsed = (System.currentTimeMillis() / 1000.0) - startTime;
    return elapsed > 3.0; // Run for 3 seconds
  }
}