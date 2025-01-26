// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorPortIDConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;
import frc.robot.Constants.CoralConstants.CoralStates;;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  SparkMax leftIntakeMotor;
  SparkMax rightIntakeMotor;
  SparkMax pivotMotor;

  SparkMaxConfig leftIntakeMotorConfig;
  SparkMaxConfig rightIntakeMotorConfig;
  SparkMaxConfig pivotMotorConfig;

  RelativeEncoder leftIntakeMotorEncoder;
  RelativeEncoder rightIntakeMotorEncoder;
  AbsoluteEncoder pivotMotorEncoder;

  PIDController pivotPIDController;
  double desiredSetpoint;

  CoralStates state;

  public CoralSubsystem() {
    leftIntakeMotor = new SparkMax(MotorPortIDConstants.kLeftAlgaeMotor, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(MotorPortIDConstants.kLeftAlgaeMotor, MotorType.kBrushless);
    pivotMotor = new SparkMax(MotorPortIDConstants.kLeftAlgaeMotor, MotorType.kBrushless);

    leftIntakeMotorEncoder = leftIntakeMotor.getEncoder();
    rightIntakeMotorEncoder = rightIntakeMotor.getEncoder();
    pivotMotorEncoder = pivotMotor.getAbsoluteEncoder();

    leftIntakeMotorConfig = new SparkMaxConfig();
    rightIntakeMotorConfig = new SparkMaxConfig();
    pivotMotorConfig = new SparkMaxConfig();

    pivotPIDController = new PIDController(0, 0, 0);

    state = CoralStates.kGround;

    configureMotors();
  }

  private void configureMotors() {}

  public void setAlgaeIntakeMotors(double voltage) {
    leftIntakeMotor.setVoltage(voltage);
    rightIntakeMotor.setVoltage(voltage);
  }

  public void setState(CoralStates state) {
    this.state = state;
    this.desiredSetpoint = state.getPivotSetpoint();
  }

  public double getPIDOutput() {
    return pivotPIDController.calculate(pivotMotorEncoder.getPosition(), desiredSetpoint);
  }

  @Override
  public void periodic() {
    pivotMotor.setVoltage(getPIDOutput());
  }
}
