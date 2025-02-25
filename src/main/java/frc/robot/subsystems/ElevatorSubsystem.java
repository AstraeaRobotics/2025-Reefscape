// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;


public class ElevatorSubsystem extends SubsystemBase {// 2 neos
  /** Creates a new ElevatorSubsystem. */
  SparkMax m_rightMotor;
  SparkMax m_leftMotor;

  // RelativeEncoder m_Encoder;
  
  ElevatorStates m_state;
  double m_SetPoint;

  RelativeEncoder elevatorEncoder;

  PIDController m_ElevatorPidController;

  ElevatorFeedforward m_elevatorFF;

  TrapezoidProfile trapezoidProfile;
  TrapezoidProfile.State goalState;
  TrapezoidProfile.State setpointState;
  // ElevatorFeedforward feedforward;

  public ElevatorSubsystem() {
    m_rightMotor = new SparkMax(7, MotorType.kBrushless); // NEED PORT # Later
    m_leftMotor = new SparkMax(8, MotorType.kBrushless);
    elevatorEncoder = m_rightMotor.getEncoder();

    // m_desiredSetPoint = 0;
    m_state = ElevatorStates.kSource; // dont have or need states right now
    m_SetPoint = m_state.getElevatorSetPoint();

    m_ElevatorPidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    m_elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kA);
    // m_rightElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kA);

    trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0)); 

    configureMotors();

  }

  public void configureMotors() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.smartCurrentLimit(35).closedLoopRampRate(8);
    rightConfig.smartCurrentLimit(35).closedLoopRampRate(8);

    m_leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setElevatorState(ElevatorStates tempState){
    m_state = tempState;
    m_SetPoint = m_state.getElevatorSetPoint();
  }

  public TrapezoidProfile.State setTrapezoidState(double sec, TrapezoidProfile.State startState, TrapezoidProfile.State endState) {
    return trapezoidProfile.calculate(sec, startState, endState); // starts at startState, ends at endState, retturns motion profile state after the number of sec inputed
  }

  public void movetoTrapezoidState(TrapezoidProfile.State state) {
    m_ElevatorPidController.calculate(elevatorEncoder.getPosition(), state.position);
  }

  public ElevatorStates getElevatorState() {
    return m_state;
  }

  public void setMotorSpeed(double speed){ // need to test which motor needs the negative
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
  }

  public void stopMotor() {
    m_rightMotor.set(0);
    m_leftMotor.set(0);
  }

  public void setElevatorVoltage(double voltage) {
    m_rightMotor.setVoltage(voltage);
    m_leftMotor.setVoltage(voltage);
  }

  public double getElevatorEncoder() {
    return elevatorEncoder.getPosition();
  }

  public double getElevatorPID() {
    return m_ElevatorPidController.calculate(getElevatorEncoder(), m_SetPoint);
  }
 
  public void setElevatorPID(){
    m_rightMotor.set(MathUtil.clamp(getElevatorPID(), -0.8, 0.8));
    m_leftMotor.set(MathUtil.clamp(getElevatorPID(), -0.8, 0.8));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Position", getElevatorEncoder());

  }
}
