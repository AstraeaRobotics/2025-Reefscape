// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Coral.IntakeCoral;
import frc.robot.commands.Coral.L1Test;
import frc.robot.commands.Coral.MoveCoralPivot;
import frc.robot.commands.Coral.SetCoralState;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final PS4Controller m_Controller = new PS4Controller(0);

  private final JoystickButton kr1 = new JoystickButton(m_Controller,PS4Controller.Button.kR1.value);
  private final JoystickButton kl1 = new JoystickButton(m_Controller,PS4Controller.Button.kL1.value);
  private final JoystickButton kCircle = new JoystickButton(m_Controller, PS4Controller.Button.kCircle.value);
  private final JoystickButton kTriangle = new JoystickButton(m_Controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kCross = new JoystickButton(m_Controller, PS4Controller.Button.kCross.value);
  private final JoystickButton kSquare = new JoystickButton(m_Controller, PS4Controller.Button.kSquare.value);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    kr1.whileTrue(new MoveCoralPivot(m_coralSubsystem, .48));
    //kr1.whileTrue(new IntakeCoral(m_coralSubsystem, 5));
    kl1.whileTrue(new IntakeCoral(m_coralSubsystem, -5));
    // kCircle.whileTrue(new IntakeCoral(m_coralSubsystem, 1));
    // kTriangle.whileTrue(new L1Test(m_coralSubsystem));
    kCircle.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kL1));
    kTriangle.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kL2));
    kCross.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kRest));
    kSquare.whileTrue(new L1Test(m_coralSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
