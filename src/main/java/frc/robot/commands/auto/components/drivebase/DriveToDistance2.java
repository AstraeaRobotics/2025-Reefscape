// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.components.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistance2 extends Command {
  /** Creates a new DriveToDistance2. */
  SwerveSubsystem m_SwerveSubsystem;
  double xDist;
  double yDist;
  double initialYaw;
  
  private PIDController xPidController = new PIDController(.576, 0, 0.00005);
  private PIDController yPidController = new PIDController(.576, 0, 0.05);

  double angle = Math.atan2(this.yDist, this.xDist);


  public DriveToDistance2(SwerveSubsystem m_SwerveSubsystem, double xDist, double yDist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.xDist = xDist;
    this.yDist = yDist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_SwerveSubsystem.resetEncoders();
    initialYaw = this.m_SwerveSubsystem.getHeading();

    this.yPidController.setSetpoint(Math.abs(this.yDist));
    this.yPidController.setTolerance(0.01);

    this.xPidController.setSetpoint(0);
    this.xPidController.setTolerance(0.01);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.xDist == 0) {
      this.m_SwerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, yPidController.calculate(this.m_SwerveSubsystem.getEncoderPosition() * Math.sin(angle)), SwerveUtil.driftCorrection(initialYaw, this.m_SwerveSubsystem.getHeading()), m_SwerveSubsystem.getHeading()), false);  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_SwerveSubsystem.drive(SwerveUtil.autoInputToChassisSpeeds(0, 0, 0, 0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.m_SwerveSubsystem.getEncoderPosition() * Math.sin(angle)) >= Math.abs(this.xDist)
    &&
    Math.abs(this.m_SwerveSubsystem.getEncoderPosition() * Math.cos(angle)) >= Math.abs(this.yDist);
}
}
