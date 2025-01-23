// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralStates;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  SparkMax coralIntakeMotor;
  SparkMax coralPivotMotor;
  AbsoluteEncoder coralPivotEncoder;
  RelativeEncoder coralIntakeEncoder;
  CoralStates m_coralState;
  double coralSetpoint;
  PIDController m_coralPidController;
  
  
  public CoralSubsystem() {
    coralIntakeMotor = new SparkMax(0, MotorType.kBrushless); // TO DO - put actual device motor IDs
    coralPivotMotor = new SparkMax(0, MotorType.kBrushless);
    coralIntakeEncoder = coralIntakeMotor.getEncoder();
    coralPivotEncoder = coralPivotMotor.getAbsoluteEncoder();
    m_coralState = CoralStates.kRest;
    coralSetpoint = m_coralState.getCoralSetpoint();
    m_coralPidController = new PIDController(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);

    configureMotors();
  }

  public void setCoralPivotMotor(double speed){
    coralPivotMotor.set(speed);
  }

  public void setCoralIntakeMotor(double speed){
    coralIntakeMotor.set(speed);
  }

  public double getCoralPivotEncoder(){
    return coralPivotEncoder.getPosition();
  }

  public double getCoralIntakeEncoder(){
    return coralIntakeEncoder.getPosition();
  }

  public void setCoralState(CoralStates tempState){
    m_coralState = tempState;
    coralSetpoint = m_coralState.getCoralSetpoint();
  }
  public CoralStates getCoralStates(){
    return m_coralState;
  }

  public double getCoralMotorPID(){
    return m_coralPidController.calculate(getCoralPivotEncoder(), coralSetpoint);
  }

  public void setCoralMotorPID(){
    coralPivotMotor.set(MathUtil.clamp(getCoralMotorPID(),-0.8,0.8));
  }

  private void configureMotors(){
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0);//TO DO find values for limit and rate
    config.closedLoopRampRate(0);

    coralIntakeMotor.configure(config, ResetMode.kResetSafeParameters.kResetSafeParameters, PersistMode.kPersistParameters.kPersistParameters);
    coralPivotMotor.configure(config, ResetMode.kResetSafeParameters.kResetSafeParameters, PersistMode.kPersistParameters.kPersistParameters);
    coralIntakeEncoder.setPosition(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Pivot Encoder", getCoralPivotEncoder());
    SmartDashboard.putNumber("Coral Intake Encoder", getCoralIntakeEncoder());
    SmartDashboard.putNumber("Coral PID Output", getCoralMotorPID());
    //setCoralMotorPID();
  }
}
