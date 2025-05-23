// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.Constants.ElevatorConstants;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;


public class ElevatorSubsystem extends SubsystemBase {// 2 neos
  /** Creates a new ElevatorSubsystem. */
  // SparkMax m_rightMotor;
  SparkMax m_leftMotor;

  // RelativeEncoder m_Encoder;
  
  ElevatorStates m_state;
  public double m_setpoint;

  RelativeEncoder elevatorEncoder;

  ProfiledPIDController m_ElevatorPidController;

  ElevatorFeedforward m_elevatorFF;

  TrapezoidProfile trapezoidProfile;

  // ElevatorFeedforward feedforward;

  public ElevatorSubsystem() {
    // m_rightMotor = new SparkMax(7, MotorType.kBrushless);
    m_leftMotor = new SparkMax(8, MotorType.kBrushless);
    elevatorEncoder = m_leftMotor.getEncoder();

    m_state = ElevatorStates.kRest; // dont have or need states right now
    m_setpoint = m_state.getElevatorSetPoint();

    m_ElevatorPidController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(100, 150));
    m_elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kA);

    configureMotors();
  }

  public void configureMotors() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.smartCurrentLimit(35).closedLoopRampRate(8);
    rightConfig.smartCurrentLimit(35).closedLoopRampRate(8);

    m_leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // m_rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetPosition();
  }

  public void setElevatorState(ElevatorStates tempState){
    m_state = tempState;
    m_setpoint = MathUtil.clamp(m_state.getElevatorSetPoint(), 1, 45);
  }

  public ElevatorStates getElevatorState() {
    return m_state;
  }

  public void setMotorSpeed(double speed){
    // m_rightMotor.set(speed);
    m_leftMotor.set(speed);
  }

  public void stopMotor() {
    // m_rightMotor.set(0);
    m_leftMotor.set(0);
  }

  public void setElevatorVoltage(double voltage) {
    // m_rightMotor.setVoltage(0);
    m_leftMotor.setVoltage(voltage);
  }

  public double getElevatorEncoder() {
    return elevatorEncoder.getPosition();
  }

  public double getElevatorPID() {
    return m_ElevatorPidController.calculate(getElevatorEncoder(), m_setpoint);
  }

  public double getElevatorOutput() {
    return MathUtil.clamp(getElevatorPID() + m_elevatorFF.calculate(0), -4, 6.5);
  }
 
  public void setElevatorPID(){
    // m_rightMotor.set(MathUtil.clamp(getElevatorPID(), -0.8, 0.8));
    m_leftMotor.set(MathUtil.clamp(getElevatorPID(), -0.8, 0.8));
  }

  public void resetPosition() {
    elevatorEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getElevatorEncoder());
    SmartDashboard.putNumber("Elevator setpoint", m_setpoint);
    setElevatorVoltage(getElevatorOutput());
  }
}
