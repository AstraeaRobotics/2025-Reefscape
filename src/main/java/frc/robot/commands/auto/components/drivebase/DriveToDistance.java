// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;

public class DriveToDistance extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_xdistanceToTravel;
  private double m_ydistanceToTravel;
  private double angle;
  private double initialYaw;
  private PIDController xController = new PIDController(.576, 0, 0.00005);
  private PIDController yController = new PIDController(.576, 0, 0.05);
  private double xDriveSpeed;
  private double yDriveSpeed;

  public DriveToDistance(SwerveSubsystem swerveSubsystem, double xdistanceToTravel, double yDistanceToTravel) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_xdistanceToTravel = xdistanceToTravel;
    this.m_ydistanceToTravel = yDistanceToTravel;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    this.m_swerveSubsystem.resetEncoders();
    initialYaw = this.m_swerveSubsystem.getHeading();
    this.angle = Math.atan2(this.m_ydistanceToTravel, this.m_xdistanceToTravel);
    this.xController.setSetpoint(Math.abs(this.m_xdistanceToTravel));
    this.xController.setTolerance(0.01);
    this.yController.setSetpoint(Math.abs(this.m_ydistanceToTravel));
    this.yController.setTolerance(0.01);
    //SmartDashboard.putNumber("Angle: ", angle);
    double speedMultiplier = 1;
    if (m_ydistanceToTravel >= 0) speedMultiplier = 0.5;
    else speedMultiplier = 1;
    // xDriveSpeed = (0.05 * m_xdistanceToTravel + 0.7) * Math.signum(m_xdistanceToTravel) * DrivebaseConstants.kAutoSpeedMultiplier;
    // yDriveSpeed = (0.05 * m_ydistanceToTravel + 0.7) * Math.signum(-m_ydistanceToTravel) * DrivebaseConstants.kAutoSpeedMultiplier * speedMultiplier;

  }

  @Override
  public void execute() {
    // -1, error = 0.05
    if (this.m_xdistanceToTravel == 0) {
      this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, yDriveSpeed, SwerveUtil.driftCorrection(initialYaw, this.m_swerveSubsystem.getHeading()), m_swerveSubsystem.getHeading()), false);  
    } 
    else if (this.m_ydistanceToTravel == 0) {
      this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(xDriveSpeed, 0, SwerveUtil.driftCorrection(initialYaw, this.m_swerveSubsystem.getHeading()), m_swerveSubsystem.getHeading()), false);
    } 
    else {
      this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(xDriveSpeed, yDriveSpeed, SwerveUtil.driftCorrection(initialYaw, this.m_swerveSubsystem.getHeading()), m_swerveSubsystem.getHeading()), false);
    }

    // SmartDashboard.putNumber("Encoder Position X: ", this.m_swerveSubsystem.getEncoderPosition() * Math.cos(angle));
    // SmartDashboard.putNumber("Encoder Position Y: ", this.m_swerveSubsystem.getEncoderPosition() * Math.sin(angle));
  }

  @Override
  public void end(boolean interrupted) {
    this.m_swerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, 0, 0), false);
  }

  @Override
  public boolean isFinished() {
    return 
      Math.abs(this.m_swerveSubsystem.getEncoderPosition() * Math.sin(angle)) >= Math.abs(this.m_xdistanceToTravel)
      &&
      Math.abs(this.m_swerveSubsystem.getEncoderPosition() * Math.cos(angle)) >= Math.abs(this.m_ydistanceToTravel);
  }
}