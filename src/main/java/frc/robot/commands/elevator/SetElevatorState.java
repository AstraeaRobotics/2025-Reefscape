// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorState extends InstantCommand {
  private ElevatorStates state;
  private ElevatorSubsystem elevatorSubsystem;
  
  public SetElevatorState(ElevatorSubsystem elevatorSub, ElevatorStates state){
    this.elevatorSubsystem = elevatorSub;
    this.state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setElevatorState(state);
  }
}