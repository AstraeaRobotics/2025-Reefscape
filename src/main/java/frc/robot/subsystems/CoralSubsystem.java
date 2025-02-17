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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralStates;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  // SparkMax coralLeftIntakeMotor;
  // SparkMax coralRightIntakeMotor;
  SparkMax coralPivotMotor;

  AbsoluteEncoder coralPivotEncoder;
  RelativeEncoder coralLeftIntakeEncoder;
  RelativeEncoder coralRightIntakeEncoder;

  CoralStates m_coralState;
  double coralSetpoint;

  PIDController m_coralPidController;
  
  ArmFeedforward m_coralPivotFeedforward;
  SimpleMotorFeedforward m_coralLeftIntakeFeedforward;
  SimpleMotorFeedforward m_coralRightIntakeFeedforward;
  
  public CoralSubsystem() {
    // coralLeftIntakeMotor = new SparkMax(1, MotorType.kBrushless); // TO DO - put actual device motor IDs
    // coralRightIntakeMotor = new SparkMax(2, MotorType.kBrushless); 
    coralPivotMotor = new SparkMax(3, MotorType.kBrushless);

    // coralLeftIntakeEncoder = coralLeftIntakeMotor.getEncoder();
    // coralRightIntakeEncoder = coralRightIntakeMotor.getEncoder();
    coralPivotEncoder = coralPivotMotor.getAbsoluteEncoder();

    m_coralState = CoralStates.kRest;
    coralSetpoint = m_coralState.getCoralSetpoint();

    m_coralPidController = new PIDController(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);

    // m_coralLeftIntakeFeedforward = new SimpleMotorFeedforward(CoralConstants.coralIntakekS, CoralConstants.coralIntakekV, CoralConstants.coralIntakekA);
    // m_coralRightIntakeFeedforward = new SimpleMotorFeedforward(CoralConstants.coralIntakekS, CoralConstants.coralIntakekV, CoralConstants.coralIntakekA);
    m_coralPivotFeedforward = new ArmFeedforward(0, CoralConstants.coralPivotkG, 0, 0);
    configureMotors();
  }

  public void setCoralPivotMotor(double speed){
    coralPivotMotor.set(speed);
  }

  // public void setCoralLeftIntakeFeedForward(double speed){
  //   coralLeftIntakeMotor.setVoltage(m_coralLeftIntakeFeedforward.calculate(speed));         
  // }

  // public void setCoralRightIntakeFeedForward(double speed){
  //   coralRightIntakeMotor.setVoltage(m_coralRightIntakeFeedforward.calculate(speed));         
  // }

  // public void setCoralPivotFeedForward(double position, double speed){
  //   coralPivotMotor.setVoltage(getCoralPivotFeedForward(position, speed));         
  // }

  // public double getCoralPivotFeedForward(double position, double speed){
  //   return m_coralPivotFeedforward.calculate(position, speed);
  // }

  // public void setLeftVoltage(double voltage){
  //   coralLeftIntakeMotor.setVoltage(voltage);         
  // }

  // public void setRightVoltage(double voltage){
  //   coralRightIntakeMotor.setVoltage(voltage);         
  // }

  // public void setIntakeVoltage(double voltage) {
  //   setLeftVoltage(voltage);
  //   setRightVoltage(voltage);
  // }

  public void setPivotVoltage(double voltage){
    coralPivotMotor.setVoltage(voltage);         
  }

  public double getPivotEncoder(){
    return (1-coralPivotEncoder.getPosition());
  }

  public void setState(CoralStates tempState){
    m_coralState = tempState;
    coralSetpoint = m_coralState.getCoralSetpoint();
  }
  public CoralStates getCoralStates(){
    return m_coralState;
  }

  public double getPivotPID(){
    return m_coralPidController.calculate(getPivotEncoder(), coralSetpoint);
  }

  public double getEncoderFF() {
    return 2 * Math.PI * getPivotEncoder() - Math.PI / 2;
  }

  public double getPivotOutput() {
    return getPivotPID() + m_coralPivotFeedforward.calculate(coralSetpoint * 2 * Math.PI - (Math.PI/2), 0);
  }

  public void setPivotPID(){
    coralPivotMotor.set(MathUtil.clamp(getPivotPID(),-0.5,0.5));
  }

  private void configureMotors(){

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig rightIntakeConfig = new SparkMaxConfig();
    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();

    pivotConfig.smartCurrentLimit(35).idleMode(IdleMode.kCoast).inverted(false);
    leftIntakeConfig.smartCurrentLimit(35);
    rightIntakeConfig.smartCurrentLimit(35).inverted(true);

    m_coralPidController.enableContinuousInput(0,1);

    // coralLeftIntakeMotor.configure(leftIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // coralRightIntakeMotor.configure(rightIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralPivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder", getPivotEncoder());
    SmartDashboard.putNumber("pivot setpoint", coralSetpoint);
    coralPivotMotor.set(MathUtil.clamp(getPivotOutput(), -.3, .3));
  }
}
