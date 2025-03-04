// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.Constants.CoralConstants.CoralStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.commands.algae.SetPivotVoltage;
import frc.robot.commands.coral.*;
import frc.robot.commands.algae.SetAlgaeState;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  private final PS4Controller m_Controller = new PS4Controller(0);
  public static final GenericHID operatorGamepad = new GenericHID(1);

  public static final JoystickButton kOperator1 = new JoystickButton(operatorGamepad, 1);
  public static final JoystickButton kOperator2 = new JoystickButton(operatorGamepad, 2);
  public static final JoystickButton kOperator3 = new JoystickButton(operatorGamepad, 3);
  public static final JoystickButton kOperator4 = new JoystickButton(operatorGamepad, 4);
  public static final JoystickButton kOperator5 = new JoystickButton(operatorGamepad, 5);
  public static final JoystickButton kOperator6 = new JoystickButton(operatorGamepad, 6);
  public static final JoystickButton kOperator7 = new JoystickButton(operatorGamepad, 7);
  public static final JoystickButton kOperator8 = new JoystickButton(operatorGamepad, 8);
  public static final JoystickButton kOperator9 = new JoystickButton(operatorGamepad, 9);
  public static final JoystickButton kOperator10 = new JoystickButton(operatorGamepad, 10);
  public static final JoystickButton kOperator11 = new JoystickButton(operatorGamepad, 11);
  public static final JoystickButton kOperator12 = new JoystickButton(operatorGamepad, 12);

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
    // m_coralSubsystem.setDefaultCommand(new IntakeCoral(m_coralSubsystem, -0.3));
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

    kL1.whileTrue(new IntakeAlgae(m_AlgaeSubsystem, -5));
    kR1.whileTrue(new IntakeAlgae(m_AlgaeSubsystem, 5));

    kCircle.onTrue(new ResetElevatorPosition(m_ElevatorSubsystem));
    // Cross is for zeroing swerve gyro

    kOperator1.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kRest), new SetCoralState(m_coralSubsystem, CoralStates.kRest), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // R
    kOperator2.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kSource), new SetCoralState(m_coralSubsystem, CoralStates.kSource), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // SRC
    kOperator3.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL1), new SetCoralState(m_coralSubsystem, CoralStates.kL1), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // CL1
    kOperator4.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL2), new SetCoralState(m_coralSubsystem, CoralStates.kL2), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // Cl2
    kOperator5.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL3), new SetCoralState(m_coralSubsystem, CoralStates.kL3), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kIn))); // Cl3
    kOperator6.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL2), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kOut), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // AL1
    kOperator7.onTrue(new ParallelCommandGroup(new SetElevatorState(m_ElevatorSubsystem, ElevatorStates.kCL3), new SetAlgaeState(m_AlgaeSubsystem, AlgaeStates.kOut), new SetCoralState(m_coralSubsystem, CoralStates.kRest))); // AL2
    kOperator8.whileTrue(new IntakeCoral(m_coralSubsystem, -5)); // IC
    kOperator9.whileTrue(new IntakeCoral(m_coralSubsystem, 5)); // EC

    // kOperator7.onTrue(new SetState(m_AlgaeSubsystem, AlgaeStates.kOut));

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
