// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LedConstants.LEDStates;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLEDoutput extends Command {
  LEDSubsystem ledSub;
  LEDStates state;
  
  /** Creates a new SetLEDoutput. */
  public SetLEDoutput(LEDSubsystem ledSub, LEDStates state) {
    this.ledSub = ledSub;
    this.state = state;
    addRequirements(ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSub.setOutPut(state);
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
