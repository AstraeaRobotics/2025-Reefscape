// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDriveKinematics kinematics;
  AHRS gyro;

  SwerveModule[] swerveModules;

  Translation2d m_frontLeftLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_frontRightLocation = new Translation2d(DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backLeftLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, DrivebaseConstants.kTrackWidth / 2);
  Translation2d m_backRightLocation = new Translation2d(-DrivebaseConstants.kWheelBase / 2, -DrivebaseConstants.kTrackWidth / 2); 

  SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  StructPublisher<Pose2d> publisher;
  StructPublisher<Pose2d> arrayPublisher;

  RobotConfig config;

  public SwerveSubsystem() {
    kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    gyro = new AHRS();

    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(2, 1, 180, "front left");
    swerveModules[1] = new SwerveModule(4, 3, 0, "front right");
    swerveModules[2] = new SwerveModule(6, 5, 180, "back left");
    swerveModules[3] = new SwerveModule(8, 7, 180, "back right");
    
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetRobotPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> drive(speeds), 
      new PPHolonomicDriveController(new PIDConstants(2.1, 0, 0), new PIDConstants(2.0, 0, 0)), 
      config,
      () -> {
      // var alliance = DriverStation.getAlliance();
      // if (alliance.isPresent()) {
      //   return alliance.get() == DriverStation.Alliance.Red;
      // }
      return false;
    },
    this);

    gyro.reset();
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    for(int i = 0; i < swerveModuleStates.length; i++){
      swerveModules[i].setState(swerveModuleStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for(int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getModulePosition();
    }
    
    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getHeading() {
    return (gyro.getYaw() + 360) % 360;
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetGyro() {
    gyro.reset();
    resetRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
  }

  public void resetRobotPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getHeading()), getModulePositions());
    SmartDashboard.putNumber("heading", getHeading());
    // LimelightHelpers.SetRobotOrientation("limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // Boolean doRejectUpdate = false;
    // if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    // {
    //   doRejectUpdate = true;
    // }
    // if(mt2.tagCount == 0)
    // {
    //   doRejectUpdate = true;
    // }
    // if(!doRejectUpdate)
    // {
    //   swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //   swerveDrivePoseEstimator.addVisionMeasurement(
    //       mt2.pose,
    //       mt2.timestampSeconds);
    // }
    
    publisher.set(getPose());
  }
}