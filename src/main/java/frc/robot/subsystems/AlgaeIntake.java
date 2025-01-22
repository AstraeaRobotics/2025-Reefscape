// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeStates;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  private final SparkMax m_pivot;
  private final SparkMax m_intake;

  private final AbsoluteEncoder m_encoder;

  private final PIDController m_pid;
  private AlgaeStates currState;
  private double desiredSetpoint;

  public AlgaeIntake() {
    m_pivot = new SparkMax(AlgaeConstants.kPivotPort, MotorType.kBrushless);
    configMotor(m_pivot);
    m_intake = new SparkMax(AlgaeConstants.kIntakePort, MotorType.kBrushless);
    configMotor(m_intake);

    m_pid = new PIDController(AlgaeConstants.kP, 0, 0);
    m_encoder = m_pivot.getAbsoluteEncoder();

    currState = AlgaeStates.kIn;
    desiredSetpoint = currState.getPivotPos();
  }

  private void configMotor(SparkMax motor){
    /*
     * This SparkMaxConfig is intended to be temporary, it would be better for a 
     * config object to be created in the constants class and imported into this class
     */

    SparkMaxConfig config = new SparkMaxConfig();
    //DUMMY VALUES
    config.smartCurrentLimit(10);
    config.closedLoopRampRate(10);
  }

  public void setPivot(double speed){m_pivot.set(speed);}
  public void setIntake(double speed){m_intake.set(speed);}
  public double getPivotEncoder(){return m_encoder.getPosition();}

  public void setState(AlgaeStates newState){
    currState = newState;
    desiredSetpoint = currState.getPivotPos();
  }

  public double getPID(){
    m_pid.setSetpoint(desiredSetpoint);
    return m_pid.calculate(getPivotEncoder());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Pivot Encoder Val", getPivotEncoder());
    setPivot(getPID());
  }
}
