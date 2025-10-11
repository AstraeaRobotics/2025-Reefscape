// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseModuleConstants;
import frc.robot.utils.SwerveUtil;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private SparkMax turnMotor;
  private SparkMax driveMotor;

  private SparkMaxConfig turnMotorConfig;
  private SparkMaxConfig driveMotorConfig;

  private PIDController turnPIDController;
  private SimpleMotorFeedforward driveFF;

  private AbsoluteEncoder turnEncoder;
  private RelativeEncoder driveEncoder;

  private int angularOffset;
  private String moduleName;

  private SwerveModuleState moduleState;

  private Boolean isInverted;

  // Simulation variables
  private double simDrivePosition = 0;
  private double simTurnPosition = 0;

  public SwerveModule(int turnMotorID, int driveMotorID, int angularOffset, String moduleName, Boolean isInverted) {
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

    turnMotorConfig = new SparkMaxConfig();
    driveMotorConfig = new SparkMaxConfig();

    turnPIDController = new PIDController(DrivebaseModuleConstants.turnKP, 0, 0);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    this.angularOffset = angularOffset;
    this.moduleName = moduleName;

    this.moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    this.isInverted = isInverted;

    driveFF = new SimpleMotorFeedforward(DrivebaseModuleConstants.driveKS, DrivebaseModuleConstants.driveKV);

    configureMotors();
  }

  public void configureMotors() {
    turnPIDController.enableContinuousInput(0, 360);

    turnMotorConfig
    .closedLoopRampRate(8)
    .smartCurrentLimit(35)
    .idleMode(IdleMode.kCoast)
    .inverted(true);
    turnMotorConfig.absoluteEncoder
    .positionConversionFactor(DrivebaseModuleConstants.kTurnEncoderPositionFactor)
    .velocityConversionFactor(DrivebaseModuleConstants.kTurnEncoderVelocityFactor)
    .inverted(true);

    turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveMotorConfig
    .closedLoopRampRate(8)
    .smartCurrentLimit(35)
    .idleMode(IdleMode.kBrake)
    .inverted(isInverted);
    driveMotorConfig.encoder
    .positionConversionFactor(DrivebaseModuleConstants.kDriveEncoderPositionFactor)
    .velocityConversionFactor(DrivebaseModuleConstants.kDriveEncoderVelocityFactor);
    
    resetEncoder();

    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getDistance(){
    if (RobotBase.isSimulation()) {
      return simDrivePosition;
    }
    return driveEncoder.getPosition();
  }

  public void resetEncoder() {
    if (RobotBase.isSimulation()) {
      simDrivePosition = 0;
    } else {
      driveEncoder.setPosition(0);
    }
  }

  public double getAngle() {
    if (RobotBase.isSimulation()) {
      return (simTurnPosition + angularOffset) % 360;
    }
    return (turnEncoder.getPosition() + angularOffset) % 360;
  }

  public SwerveModulePosition getModulePosition() {
    double distance = getDistance();
    Rotation2d angle = Rotation2d.fromDegrees(getAngle());
    return new SwerveModulePosition(distance, angle);
  }

  public SwerveModuleState getModuleState() {
    return moduleState;
  }

  public void setState(SwerveModuleState state, boolean slowMode) {
    moduleState = state;
    drive(slowMode);
  }

  public void drive(boolean slowMode) {
    double[] optimizedModule = SwerveUtil.optimizeModule(getAngle(), moduleState.angle.getDegrees() + 180, moduleState.speedMetersPerSecond);

    if (RobotBase.isSimulation()) {
      // Update simulated turn position - make it faster for simulation
      double targetAngle = optimizedModule[0];
      double angleDiff = targetAngle - simTurnPosition;
      
      // Normalize angle difference to [-180, 180]
      while (angleDiff > 180) angleDiff -= 360;
      while (angleDiff < -180) angleDiff += 360;
      
      // Move towards target angle (faster in sim)
      simTurnPosition += angleDiff * 0.3; // Increased from 0.1
      simTurnPosition = (simTurnPosition + 360) % 360;

      // Update simulated drive position based on speed
      double speed = optimizedModule[1]; // Already in m/s
      if (slowMode) speed /= 2;
      
      // Integrate velocity to get position (speed * time)
      double deltaPosition = speed * 0.02; // 20ms = 0.02s loop time
      simDrivePosition += deltaPosition;
      
      // DEBUG
      if (Math.abs(speed) > 0.01) {
        System.out.println(moduleName + " driving - Speed: " + speed + " Delta: " + deltaPosition + " Total: " + simDrivePosition);
      }
      
    } else {
      // Real robot control
      turnMotor.set(-turnPIDController.calculate(getAngle(), optimizedModule[0]));
      driveMotor.setVoltage(MathUtil.clamp(slowMode ? driveFF.calculate(optimizedModule[1] / 2) : driveFF.calculate(optimizedModule[1]), -6, 6));
    }
  }

  public double getVelocity() {
    if (RobotBase.isSimulation()) {
      return moduleState.speedMetersPerSecond;
    }
    return driveEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // Debug output in simulation
    if (RobotBase.isSimulation()) {
      SmartDashboard.putNumber(moduleName + " Sim Angle", simTurnPosition);
      SmartDashboard.putNumber(moduleName + " Sim Distance", simDrivePosition);
      SmartDashboard.putNumber(moduleName + " Desired Speed", moduleState.speedMetersPerSecond);
      SmartDashboard.putNumber(moduleName + " Desired Angle", moduleState.angle.getDegrees());
    }
  }
}