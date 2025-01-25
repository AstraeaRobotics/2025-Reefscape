// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorSpeed extends Command {
  /** Creates a new elevatorSpeed. */
  double m_elevatorSpeed;
  ElevatorSubsystem m_elevatorSubsystem;

  public elevatorSpeed(double speed, ElevatorSubsystem sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSpeed = speed;
    m_elevatorSubsystem = sub;

    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setMotorSpeed(m_elevatorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   double position = m_elevatorSubsystem.getMotorPostion();
  //   if(m_elevatorSpeed < 0 && m_motors.getLimitSwitch()) {
  //     return true;
  //   } 
  //   else if (m_elevatorSpeed >= 0 && position > 0) {
  //     return true;
  //   }
  //   else {
  //     return false;
  //   }
  // }
    return false;
  }

}
