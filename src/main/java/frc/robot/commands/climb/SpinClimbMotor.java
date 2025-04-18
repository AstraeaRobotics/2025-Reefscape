// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climb;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimbSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class SpinClimbMotor extends Command {
//   /** Creates a new SpinClimbManual. */
//   ClimbSubsystem m_ClimbSubsystem;
//   double speed;
//   public SpinClimbMotor(ClimbSubsystem climbSubsystem, double speed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_ClimbSubsystem = climbSubsystem;
//     this.speed = speed;
//     addRequirements(m_ClimbSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_ClimbSubsystem.setPivotVoltage(speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_ClimbSubsystem.setPivotVoltage(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
