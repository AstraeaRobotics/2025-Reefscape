// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kP = 1.1; // TO DO - tune PID
    public static final double kI = 0.0;
    public static final double kD = 0.08;

    //feed forward constants 
    public static final double coralIntakekS = 0.0;//TO DO - find feed forward values
    public static final double coralIntakekV = 0.0;
    public static final double coralIntakekA = 0.0;

    public static final double coralPivotkS = 0.0;
    public static final double coralPivotkG = 0.07; // 0.48, .07
    public static final double coralPivotkV = 0.0;
    public static final double coralPivotkA = 0.0;

    public enum CoralStates{
      kRest(0.823),
      kL1(.027),  //TO DO - find encoder values
      kL2(.98),
      kL3(0.961),
      kSource(0.85);
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
    public static final double kP = 0.001;
    
    public static final int kIntakePortL = 4;
    public static final int kIntakePortR = 5;

    public static final double intakeKS = 0;
    public static final double intakeKV = 0;
    public static final double intakeKA = 0;
    
    public static final double pivotKG = 2.5;
  
    
    public enum AlgaeStates{
      kOut(0.25),
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
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0.33; // mass of elevator * accel of gravity
    public static final double kV = 0;
    public static final double kA = 0;
    
    public enum ElevatorStates{  // Coral 1,2,3,4, Alage: 2,3 Proccesser, Source
      kRest(0),
      kSource(8.8), // needed to test for these values
      kProcessor(21),
      kL1(8.8),
      kL2(24),
      kL3(40);

      private double elevatorSetPoint;

      private ElevatorStates(double elevatorSetPoint) {
        this.elevatorSetPoint = elevatorSetPoint;
      }
      
      public double getElevatorSetPoint(){
        return elevatorSetPoint;
      }
    }

  
  }
}
