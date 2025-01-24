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
  public static class ElevatorConstants{
    //public static final double kEncoderConversionFactor = 2 *Math.PI * 2;
    public static final double kEncoderConversionFactor = 2*Math.PI;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public enum ElevatorStates{
      kGround(0),
      kHalf(50),
      kFull(100);
      

      
      private double elevatorSetPoint;
      private ElevatorStates(double elevatorSetPoint){
      this.elevatorSetPoint = elevatorSetPoint;
      }
      
    


      public double getElevatorSetPoint(){
        return elevatorSetPoint;
      }


    }

  
  }
}
