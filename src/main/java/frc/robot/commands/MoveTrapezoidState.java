// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTrapezoidState extends Command {
  /** Creates a new MoveTrapezoidState. */
  ElevatorSubsystem elevatorsub;
  TrapezoidProfile.State intialState;
  TrapezoidProfile.State endState;
  double sec;
  public MoveTrapezoidState(ElevatorSubsystem sub, TrapezoidProfile.State state1, TrapezoidProfile.State state2, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevatorsub = sub;
    intialState = state1;
    endState = state2;
    sec = s;

    addRequirements(sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorsub.movetoTrapezoidState(elevatorsub.setTrapezoidState(sec, intialState, endState));
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
