// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class MotorPortIDConstants {
        public static final int kLeftAlgaeMotor = 0;
        public static final int kRightAlgaeMotor = 0;
        public static final int kPivotAlgaeMotor = 0;
    }

    public static class DrivebaseModuleConstants {
        // Physical Constants
        public static final double kDriveGearRatio = 3.56;
        public static final double kWheelDiameter = Units.inchesToMeters(3);

        // Conversion Factors
        public static final int kTurnEncoderPositionFactor = 360;
        public static final int kTurnEncoderVelocityFactor = 60; // not sure about this

        public static final double kDriveEncoderPositionFactor = (1 / kDriveGearRatio) * 2 * Math.PI * (kWheelDiameter / 2);
        public static final double kDriveEncoderVelocityFactor = 1/(60 * kDriveGearRatio);

        // PID Constants (change later)
        public static final double turnKP = 0.004;
        public static final double turnKI = 0;
        public static final double turnKD = 0;

        public static final double driveKP = 0.01;
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        // FeedForward Constants
        public static final double turnKV = 0.00005; // 0.31
        public static final double driveKV = 2.0; // 6.5
  }

  public static class DrivebaseConstants {
    // Physical constants
    public static final double kWheelBase = Units.inchesToMeters(20);/* the distance between the front and rear wheels */
    public static final double kTrackWidth = Units.inchesToMeters(22); /* the distance between left and right wheels */
  }

  public static class ElevatorConstants {
    public static final double kGroundSetpoint = 0;
    public static final double KL1Setpoint = 0;
    public static final double kL2Setpoint = 0;
    public static final double kL3Setpoint = 0;
    public static final double KL4Setpoint = 0;
    public static final double kSourceSetpoint = 0;
    public static final double kProcessorSetpoint = 0;

    public enum ElevatorStates {
        kGround(kGroundSetpoint),
        KL1(KL1Setpoint),
        KL2(kL2Setpoint),
        KL3(kL3Setpoint),
        KL4(KL4Setpoint),
        kSource(kSourceSetpoint),
        kProcessor(kProcessorSetpoint);

        private double elevatorSetpoint;

        private ElevatorStates(double elevatorSetpoint) {
            this.elevatorSetpoint = elevatorSetpoint;
        }
        
        public double getElevatorSetpoint() {
            return elevatorSetpoint;
        }
    }
  }
}
