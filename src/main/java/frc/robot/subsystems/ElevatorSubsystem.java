// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax m_rightMotor;
  SparkMax m_leftMotor;

  RelativeEncoder m_Encoder;
  DigitalInput m_limitSwitchMIN;
  double m_desiredSetPoint;

  ElevatorStates m_state;
  PIDController m_ElevatorPidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  public ElevatorSubsystem(SparkMax motor1, SparkMax motor2) {
    m_rightMotor = motor1;
    m_leftMotor = motor2;
    m_Encoder = m_rightMotor.getEncoder();
    m_desiredSetPoint = 0;
    m_state = ElevatorStates.kGround;
  }
  public void setSpeed(double speed){
    m_rightMotor.set(speed);
    m_leftMotor.set(-speed);
  }

  public void configureMotors() {
    SparkMaxConfig configElevatorL = new SparkMaxConfig();
    SparkMaxConfig configElevatorR = new SparkMaxConfig();

    configElevatorL.smartCurrentLimit(0); //arbitary #
    configElevatorL.closedLoopRampRate(0); //arbitary #
    configElevatorL.inverted(true);

    m_leftMotor.configure(configElevatorR, null, null);
    m_rightMotor.configure(configElevatorR, null, null);

    configElevatorR.smartCurrentLimit(0);
    configElevatorR.closedLoopRampRate(0);



  }

  public double getEncoder(){
    return m_Encoder.getPosition();
 }
 public void ElevatorFeedforward(double Velocity, double acceleration) {
  setSpeed(feedforward.calculate(Velocity,acceleration));
}
 public boolean getLimitSwitch() {
  return m_limitSwitchMIN.get();
 }

 public void setState(ElevatorStates tempState){
  m_state = tempState;
  m_desiredSetPoint = m_state.getElevatorSetPoint();
 }

 public double getMotorPID() {
  return m_ElevatorPidController.calculate(getEncoder());
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSpeed(m_ElevatorPidController.calculate(getEncoder(), m_desiredSetPoint));
    SmartDashboard.putNumber("Elevator Encoder", m_Encoder.getPosition());
    SmartDashboard.putNumber("Elevator Speed", m_Encoder.getVelocity());
    SmartDashboard.putNumber("PID Output", getMotorPID());

  }
}
