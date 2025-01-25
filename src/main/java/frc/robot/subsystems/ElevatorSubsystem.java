// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.motorcontrol.Talon; //Talon imports, automatically has PID control
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

 

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  TalonFX m_rightMotor;
  TalonFX m_leftMotor;

  TalonFXConfiguration configs;

  Slot0Configs rightMotorConfigs;
  Slot1Configs leftMotorConfigs;

  // RelativeEncoder m_Encoder;
  double m_desiredSetPoint;

  ElevatorStates m_state;
  PIDController m_ElevatorPidController;
  // ElevatorFeedforward feedforward;

  VelocityVoltage m_leftVelocityVoltage;
  VelocityVoltage m_rightVelocityVoltage;

  NeutralOut m_brake;

  public ElevatorSubsystem() {
    m_rightMotor = new TalonFX(0); // NEED PORT # Later
    m_leftMotor = new TalonFX(0);
    //m_Encoder = m_rightMotor.getEncoder();

    // m_desiredSetPoint = 0;
    // m_state = ElevatorStates.kGround; // dont have or need states right now

    m_ElevatorPidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
    // feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    rightMotorConfigs = new Slot0Configs();
    leftMotorConfigs = new Slot1Configs();
    configs = new TalonFXConfiguration();

    m_brake = new NeutralOut();

    m_rightVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    m_leftVelocityVoltage = new VelocityVoltage(0).withSlot(1);


    configureMotors();

  }

  public void configureMotors() {
    rightMotorConfigs.kP = 0; //Need these values
    rightMotorConfigs.kI = 0;
    rightMotorConfigs.kD = 0;
    rightMotorConfigs.kS = 0;
    rightMotorConfigs.kG = 0;
    rightMotorConfigs.kA = 0;
    rightMotorConfigs.kV = 0;

    leftMotorConfigs.kP = 0;
    leftMotorConfigs.kI = 0;
    leftMotorConfigs.kD = 0;
    leftMotorConfigs.kS = 0;
    leftMotorConfigs.kG = 0;
    leftMotorConfigs.kA = 0;
    leftMotorConfigs.kV = 0;

    m_leftMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rightMotor.setNeutralMode(NeutralModeValue.Coast);

    m_leftMotor.getConfigurator().apply(leftMotorConfigs);
    m_rightMotor.getConfigurator().apply(rightMotorConfigs);

  }

  public void setState(ElevatorStates tempState){
    m_state = tempState;
    m_desiredSetPoint = m_state.getElevatorSetPoint();
  }

  public void setMotorSpeed(double speed){ // need to test which motor needs the negative
    // m_rightMotor.set(speed);
    // m_leftMotor.set(-speed);
    m_rightMotor.setControl(m_rightVelocityVoltage.withVelocity(speed));
    m_leftMotor.setControl(m_leftVelocityVoltage.withVelocity(speed));
  }

  public void stopMotor() {
    m_rightMotor.setControl(m_brake);
    m_leftMotor.setControl(m_brake);
  }

  public void setMotorCoast() {
    m_rightMotor.setNeutralMode(NeutralModeValue.Coast);
    m_leftMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getMotorPostion() {
    return m_rightMotor.getPosition().getValueAsDouble();
  }

  public double getMotorVelocity() {
    return m_rightMotor.getVelocity().getValueAsDouble();
  }

  // public double getEncoder() {
  //   return m_Encoder.getPosition();
  // }
  
  // public void ElevatorFeedforward(double Velocity, double acceleration) {
  //   setSpeed(feedforward.calculate(Velocity,acceleration));
  // }
  

  public double getMotorP() {
    return rightMotorConfigs.kP;
  }

  public double getMotorI() {
    return rightMotorConfigs.kI;
  }

  public double getMotorD() {
    return rightMotorConfigs.kD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setSpeed(m_ElevatorPidController.calculate(getEncoder(), m_desiredSetPoint));
    SmartDashboard.putNumber("Elevator Encoder Position", getMotorPostion());
    SmartDashboard.putNumber("Elevator Speed", getMotorVelocity());
    SmartDashboard.putNumber("Elevator TalonMotor kP", getMotorP());
    SmartDashboard.putNumber("Elevator TalonMotor kI", getMotorI());
    SmartDashboard.putNumber("Elevator TalonMotor kD", getMotorD());
  }
}
