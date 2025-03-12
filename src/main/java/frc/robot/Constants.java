// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class CoralConstants {
    public static final double kP = 13.0; // TO DO - tune PID - 1.2
    public static final double kI = 0.0;
    public static final double kD = 1.0; // 0.25

    //feed forward constants 
    public static final double coralIntakekS = 0.0;//TO DO - find feed forward values
    public static final double coralIntakekV = 0.0;
    public static final double coralIntakekA = 0.0;

    public static final double coralPivotkS = 0.0;
    public static final double coralPivotkG = 0.57; // 0.045
    public static final double coralPivotkV = 0.0;
    public static final double coralPivotkA = 0.0;

    public enum CoralStates{
      kRest(0.823),
      kL1(0.97),  //TO DO - find encoder values
      kL2(.99),
      kL3(0.95),
      kSource(0.83);
      private double coralSetpoint;

      private CoralStates(double coralSetpoint){
        this.coralSetpoint = coralSetpoint;
      }

      public double getCoralSetpoint(){
        return coralSetpoint;
      }
    }
  }

  public static class AlgaeConstants {
    public static final int kPivotPort = 6;
    public static final double kP = 10.0;
    
    public static final int kIntakePortL = 4;
    public static final int kIntakePortR = 5;

    public static final double intakeKS = 0;
    public static final double intakeKV = 0;
    public static final double intakeKA = 0;
    
    public static final double pivotKG = 1.5;
  
    
    public enum AlgaeStates{
      kL1(0.15),
      kL2(0.25),
      kL3(0.35),
      kIn(0);

      private double kPivotPos;

      private AlgaeStates(double pivotpos){
        kPivotPos = pivotpos;
      }

      public double getPivotPos(){
        return kPivotPos;
      }
    }
  }
  public static class ElevatorConstants{
    //public static final double kEncoderConversionFactor = 2 *Math.PI * 2;
    public static final double kEncoderConversionFactor = 2*Math.PI;
    public static final double kP = 1.4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0.33;
    public static final double kV = 0;
    public static final double kA = 0;
    
    public enum ElevatorStates{  // Coral 1,2,3,4, Alage: 2,3 Proccesser, Source
      kRest(0),
      kSource(10), // needed to test for these values
      kProcessor(21),
      kCL1(8),
      kCL2(21.5),
      kCL3(43.5),
      kAl1(10),
      kAL2(26),
      kAl3(43);


      private double elevatorSetPoint;

      private ElevatorStates(double elevatorSetPoint) {
        this.elevatorSetPoint = elevatorSetPoint;
      }
      
      public double getElevatorSetPoint(){
        return elevatorSetPoint;
      }
    }
  }

  public static class DrivebaseModuleConstants {
        // Physical Constants
        public static final double kDriveGearRatio = 3.56;
        public static final double kWheelDiameter = Units.inchesToMeters(3);
        public static final double kMaxDriveVoltage = 6.0;

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
        // public static final double turnKV = 0.00005; // 0.31
        public static final double driveKV = 6.5; // 6.5
        public static final double driveKS = 0;
  }

  public static class DrivebaseConstants {
    // Physical constants
  
   
    // public static final double kWheelBase = Units.inchesToMeters(20);/* the distance between the front and rear wheels */
    // public static final double kTrackWidth = Units.inchesToMeters(22); /* the distance between left and right wheels */

     /* ^^^^^
        lyra  
      */

    
    // 2025 nameless bot constants
    public static final double kWheelBase = Units.inchesToMeters(26.125);
    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    public static final double kAutoSpeedMultiplier = 0.8;

  }
}
