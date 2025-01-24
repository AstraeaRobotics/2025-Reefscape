// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
package frc.robot.subsystems;

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

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax m_rightMotor;
  private final SparkMax m_leftMotor;
  private final RelativeEncoder m_Encoder;
  DigitalInput m_limitSwitchMIN;
  double m_desiredSetPoint;
  ElevatorStates m_state;
  PIDController pid = new PIDController(0.1, 0.3, 0.1);
  public ElevatorSubsystem(SparkMax motor1, SparkMax motor2) {
    m_rightMotor = motor1;
    m_leftMotor = motor2;
    m_Encoder = sparkMaxOne.getEncoder();
    m_desiredSetPoint = 0;
    m_state = ElevatorStates.kGround;
  }
  public void setSpeed(double speed){
    m_rightMotor.set(speed);
    m_leftMotor.set(-speed);
  }

  public double getEncoder(){
    return m_Encoder.getPosition();
 }
 
 public boolean getLimitSwitch() {
  return m_limitSwitchMIN.get();
 }

 public void setState(ElevatorStates tempState){
  m_state = tempState;
  m_desiredSetPoint = m_state.getElevatorSetPoint();
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSpeed(pid.calculate(getEncoder(), m_desiredSetPoint));
  }
}
