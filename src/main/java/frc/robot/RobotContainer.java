// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.commands.SetClimbState;
// import frc.robot.commands.SpinClimbManual;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants.ClimbStates;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final PS4Controller driverController = new PS4Controller(0);

  public static final JoystickButton kCircle = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);
  public static final JoystickButton kTriangle = new JoystickButton(driverController, PS4Controller.Button.kTriangle.value);
  public static final JoystickButton kSquare = new JoystickButton(driverController,PS4Controller.Button.kSquare.value);
  public static final JoystickButton kCross = new JoystickButton(driverController,PS4Controller.Button.kCross.value);

  public static final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    // kCircle.whileTrue(new SpinClimbManual(m_climbSubsystem, .01));
    // kTriangle.whileTrue(new SpinClimbManual(m_climbSubsystem, -.01));
    kCross.onTrue(new SetClimbState(m_climbSubsystem, ClimbStates.kTop));
    kSquare.onTrue(new SetClimbState(m_climbSubsystem, ClimbStates.kGround));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
