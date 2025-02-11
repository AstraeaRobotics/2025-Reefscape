// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Algae.SetAlgaeIntakeVoltageManually;
import frc.robot.commands.Algae.SetAlgaePivotVoltageManually;
import frc.robot.subsystems.AlgaeIntake;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake();

  private final PS4Controller m_controller = new PS4Controller(OperatorConstants.kDriverControllerPort);

  private final JoystickButton kTriangle = new JoystickButton(m_controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kCircle = new JoystickButton(m_controller, PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_controller, PS4Controller.Button.kSquare.value);
  private final JoystickButton kCross = new JoystickButton(m_controller, PS4Controller.Button.kCross.value);

  public RobotContainer() {
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

    kTriangle.whileTrue(new SetAlgaeIntakeVoltageManually(m_algaeIntake, 1));
    kCircle.whileTrue(new SetAlgaePivotVoltageManually(m_algaeIntake, 1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

  // }
}
