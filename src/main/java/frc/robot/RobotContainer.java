// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.commands.Coral.IntakeCoral;
// import frc.robot.commands.Coral.IntakeCoral;
// import frc.robot.commands.Coral.L1Test;
import frc.robot.commands.Coral.MoveCoralPivot;
import frc.robot.commands.Coral.SetCoralState;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.commands.algae.SetPivotVoltage;
import frc.robot.commands.algae.SetState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.ResetElevatorPosition;
import frc.robot.commands.SetElevatorState;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  // private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS4Controller m_Controller = new PS4Controller(0);

  private final JoystickButton kCircle = new JoystickButton(m_Controller,PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_Controller, PS4Controller.Button.kSquare.value);
  private final JoystickButton kCross = new JoystickButton(m_Controller, PS4Controller.Button.kCross.value);
  private final JoystickButton kTriangle = new JoystickButton(m_Controller, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kR1 = new JoystickButton(m_Controller,PS4Controller.Button.kR1.value);
  private final JoystickButton kR2 = new JoystickButton(m_Controller,PS4Controller.Button.kR2.value);
  private final JoystickButton kL1 = new JoystickButton(m_Controller,PS4Controller.Button.kL1.value);
  private final JoystickButton kL2 = new JoystickButton(m_Controller,PS4Controller.Button.kL2.value);
  

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // m_AlgaeSubsystem.setDefaultCommand(new IntakeAlgae(m_AlgaeSubsystem, -.25));
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
    // kR1.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kSource));
    // kR2.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kL1));
    // kr1.whileTrue(new MoveCoralPivot(m_coralSubsystem, 3));
    // kCircle.onTrue(new SetCoralState(m_coralSubsystem, CoralStates.kL2));

    // kR1.whileTrue(new MoveElevator(m_ElevatorSubsystem, 5));
    // kL1.whileTrue(new MoveElevator(m_ElevatorSubsystem, -1));
    kL1.whileTrue(new IntakeCoral(m_coralSubsystem, 5));
    kR1.whileTrue(new IntakeCoral(m_coralSubsystem, -5));
    kCircle.onTrue(new ResetElevatorPosition(m_ElevatorSubsystem));
    kTriangle.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest), new SetCoralState(m_coralSubsystem, CoralStates.kRest)));
    kSquare.onTrue(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kSource));
    kCross.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kL3), new SetCoralState(m_coralSubsystem, CoralStates.kL2)));
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
