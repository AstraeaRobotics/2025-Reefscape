// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

  // private final SendableChooser<Command> autoChooser;

  private final PS4Controller m_driverController = new PS4Controller(0);
  private final JoystickButton kCross = new JoystickButton(m_driverController, PS4Controller.Button.kCross.value);
  private final JoystickButton kCircle = new JoystickButton(m_driverController, PS4Controller.Button.kCircle.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_SwerveSubsystem.setDefaultCommand(new TeleopSwerve(m_SwerveSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX, m_driverController::getR2Axis));
    
    // autoChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    // NamedCommands.registerCommand("ElevatorL4", new ElevatorL4(m_ElevatorSubsystem));
    // NamedCommands.registerCommand("ElevatorL3", new ElevatorL3(m_ElevatorSubsystem));

    configureBindings();
  }


  private void configureBindings() {
    kCross.onTrue(new ResetGyro(m_SwerveSubsystem));
    kCircle.onTrue(new PathPlannerAuto("test auto"));
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
