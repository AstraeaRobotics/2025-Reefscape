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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CoralStates;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  SparkMax coralLeftIntakeMotor;
  SparkMax coralRightIntakeMotor;
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
    coralLeftIntakeMotor = new SparkMax(0, MotorType.kBrushless); // TO DO - put actual device motor IDs
    coralRightIntakeMotor = new SparkMax(0, MotorType.kBrushless); 
    coralPivotMotor = new SparkMax(0, MotorType.kBrushless);

    coralLeftIntakeEncoder = coralLeftIntakeMotor.getEncoder();
    coralRightIntakeEncoder = coralRightIntakeMotor.getEncoder();
    coralPivotEncoder = coralPivotMotor.getAbsoluteEncoder();

    m_coralState = CoralStates.kRest;
    coralSetpoint = m_coralState.getCoralSetpoint();

    m_coralPidController = new PIDController(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);

    m_coralLeftIntakeFeedforward = new SimpleMotorFeedforward(CoralConstants.coralIntakekS, CoralConstants.coralIntakekV, CoralConstants.coralIntakekA);
    m_coralRightIntakeFeedforward = new SimpleMotorFeedforward(CoralConstants.coralIntakekS, CoralConstants.coralIntakekV, CoralConstants.coralIntakekA);
    m_coralPivotFeedforward = new ArmFeedforward(0, CoralConstants.coralPivotkG, 0, 0);
    configureMotors();
  }

  public void setCoralPivotMotor(double speed){
    coralPivotMotor.set(speed);
  }

  public void setCoralLeftIntakeFeedForward(double speed){
    coralLeftIntakeMotor.setVoltage(m_coralLeftIntakeFeedforward.calculate(speed));         
  }

  public void setCoralRightIntakeFeedForward(double speed){
    coralRightIntakeMotor.setVoltage(m_coralRightIntakeFeedforward.calculate(speed));         
  }

  public void setCoralPivotFeedForward(double position, double speed){
    coralPivotMotor.setVoltage(getCoralPivotFeedForward(position, speed));         
  }

  public double getCoralPivotFeedForward(double position, double speed){
    return m_coralPivotFeedforward.calculate(position, speed);

  }

  public void setLeftCoralIntakeVoltage(double voltage){
    coralLeftIntakeMotor.setVoltage(voltage);         
  }

  public void setRightCoralIntakeVoltage(double voltage){
    coralRightIntakeMotor.setVoltage(voltage);         
  }

  public void setcoralPivotVoltage(double voltage){
    coralPivotMotor.setVoltage(voltage);         
  }

  public void setLeftCoralIntakeMotor(double speed){
    coralLeftIntakeMotor.set(speed);
  }

  public void setRightCoralIntakeMotor(double speed){
    coralRightIntakeMotor.set(speed);
  }

  public double getCoralPivotEncoder(){
    return coralPivotEncoder.getPosition();
  }

  public double getLeftCoralIntakeEncoder(){
    return coralLeftIntakeEncoder.getPosition();
  }

  public double getRightCoralIntakeEncoder(){
    return coralRightIntakeEncoder.getPosition();
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

  public void setCoralMotorPID(double position){
    coralPivotMotor.set(MathUtil.clamp((getCoralMotorPID()+ getCoralPivotFeedForward(position, 0)),-0.8,0.8));
  }

  private void configureMotors(){
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig rightIntakeConfig = new SparkMaxConfig();
    SparkMaxConfig leftIntakeConfig = new SparkMaxConfig();
    //config.smartCurrentLimit(0);//TO DO find values for limit and rate
    //config.closedLoopRampRate(0);
<<<<<<< HEAD
    pivotConfig.smartCurrentLimit(35).closedLoopRampRate(0);
    intakeConfig.smartCurrentLimit(35).closedLoopRampRate(0);
=======
    pivotConfig.smartCurrentLimit(0).closedLoopRampRate(0);
    rightIntakeConfig.smartCurrentLimit(0).closedLoopRampRate(0);
    leftIntakeConfig.smartCurrentLimit(0).closedLoopRampRate(0);
<<<<<<< Updated upstream
=======
>>>>>>> 2d602620bfe5a00207fd21a4f919e8040b28a8e2
>>>>>>> Stashed changes

    coralLeftIntakeMotor.configure(leftIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralRightIntakeMotor.configure(rightIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralPivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Pivot Encoder", getCoralPivotEncoder());
    SmartDashboard.putNumber("Coral PID Output", getCoralMotorPID());
    SmartDashboard.putNumber("Coral Intake Motor Velocity", coralLeftIntakeEncoder.getVelocity());
    SmartDashboard.putNumber("Coral Intake Motor Velocity", coralRightIntakeEncoder.getVelocity());
    SmartDashboard.putNumber("Coral Pivot Motor Velocity", coralPivotEncoder.getVelocity());

    //setCoralMotorPID();
  }
}
