// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  private final SparkMax m_pivot;
  private final SparkMax m_intakeL;
  private final SparkMax m_intakeR;

  private final AbsoluteEncoder m_pivotEncoder;
  private final RelativeEncoder m_intakeEncoder;
  

  private final PIDController m_pid;
  private final SimpleMotorFeedforward m_intakeFeedForward;
  private final SimpleMotorFeedforward m_pivotFeedforward;
  private AlgaeStates currState;
  private double desiredSetpoint;

  public AlgaeIntake() {
    m_pivot = new SparkMax(AlgaeConstants.kPivotPort, MotorType.kBrushless);
    m_intakeL = new SparkMax(AlgaeConstants.kIntakePortL, MotorType.kBrushless);
    m_intakeR = new SparkMax(AlgaeConstants.kIntakePortR, MotorType.kBrushless);

    m_pid = new PIDController(AlgaeConstants.kP, 0, 0);
    m_pivotEncoder = m_pivot.getAbsoluteEncoder();
    m_intakeEncoder = m_intakeL.getEncoder();
    m_intakeEncoder.setPosition(0);

    configMotors();

    currState = AlgaeStates.kIn;
    desiredSetpoint = currState.getPivotPos();

    m_intakeFeedForward = new SimpleMotorFeedforward(AlgaeConstants.intakeKS, AlgaeConstants.intakeKV, AlgaeConstants.intakeKA);
    m_pivotFeedforward = new SimpleMotorFeedforward(AlgaeConstants.pivotKS, AlgaeConstants.pivotKV, AlgaeConstants.pivotKA);
  }

  private void configMotors(){
    /*
     * This SparkMaxConfig is intended to be temporary, it would be better for a 
     * config object to be created in the constants class and imported into this class
     */

    SparkMaxConfig configPivot = new SparkMaxConfig();
    
    configPivot.smartCurrentLimit(10);
    configPivot.closedLoopRampRate(10);

    m_pivot.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configIntakeL = new SparkMaxConfig();

    configIntakeL.smartCurrentLimit(10);
    configIntakeL.closedLoopRampRate(10);

    m_intakeL.configure(configIntakeL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig configIntakeR = new SparkMaxConfig();

    configIntakeR.smartCurrentLimit(10);
    configIntakeR.closedLoopRampRate(10);
    configIntakeR.inverted(true);

    m_intakeR.configure(configIntakeR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPivotEncoder(){return m_pivotEncoder.getPosition();}

  public double getIntakeVelocity(){return m_intakeEncoder.getVelocity();}

  public double getPivotVelocity(){return m_pivotEncoder.getVelocity();}

  public double getPID(){ return m_pid.calculate(getPivotEncoder(), desiredSetpoint);}

  public void setIntakeManual(double speed){m_intakeR.set(speed); m_intakeL.set(speed);}

  public void setPivotManual(double speed){m_pivot.set(speed);}

  public void setState(AlgaeStates newState){
    currState = newState;
    desiredSetpoint = currState.getPivotPos();
  }

  public void setIntake(double speed){
    m_intakeR.setVoltage(m_intakeFeedForward.calculate(speed));
    m_intakeL.setVoltage(m_intakeFeedForward.calculate(speed));
  }

  public void setPivot(double speed){
    m_pivot.setVoltage(m_pivotFeedforward.calculate(speed));
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Pivot Encoder Val", getPivotEncoder());
    SmartDashboard.putNumber("Algae PID output", getPID());
    SmartDashboard.putNumber("Algae Intake Velocity", getIntakeVelocity());
    SmartDashboard.putNumber("Algae Pivot Velocity", getPivotEncoder());
    //setPivot(getPID());
  }
}
