// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax leftElevatorMotor;
  SparkMax rightElevatorMotor;

  RelativeEncoder elevatorEncoder;

  SparkMaxConfig leftElevatorMotorConfig;
  SparkMaxConfig rightElevatorMotorConfig;

  PIDController elevatorPIDController;

  double setpoint;
  ElevatorStates state;

  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkMax(MotorPortIDConstants.kPivotAlgaeMotor, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(MotorPortIDConstants.kPivotAlgaeMotor, MotorType.kBrushless);

    leftElevatorMotorConfig = new SparkMaxConfig();
    rightElevatorMotorConfig = new SparkMaxConfig();

    elevatorEncoder = leftElevatorMotor.getEncoder();

    elevatorPIDController = new PIDController(0, 0, 0);

    state = ElevatorStates.kGround;
    setpoint = state.getElevatorSetpoint();
  }

  public void setState(ElevatorStates state) {
    this.state = state;
    setpoint = state.getElevatorSetpoint();
  }

  public double getPIDOutput() {
    return elevatorPIDController.calculate(elevatorEncoder.getPosition(), setpoint);
  }

  private void configureMotors() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftElevatorMotor.setVoltage(getPIDOutput());
    rightElevatorMotor.setVoltage(getPIDOutput());
  }
}
