// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.utils.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationalAlign extends Command {

  ClimbSubsystem climbSubsystem;
  PIDController pidController;
  Limelight limelight;
  double centerPoint;
  double tolerance = 0.5;
  /** Creates a new RotationalAlign. */
  public RotationalAlign(ClimbSubsystem climbSubsystem, double centerPoint) {

    this.climbSubsystem = climbSubsystem;
    this.pidController = new PIDController(0.1, 0.0, 0.0);
    this.limelight = new Limelight();
    this.pidController = new PIDController(0.1, 0.0, 0.0);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(centerPoint);
    pidController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(climbSubsystem.getLimelight().getTx());
    climbSubsystem.spinClimbMotorPosition(output); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.spinClimbMotorPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
