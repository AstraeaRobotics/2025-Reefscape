// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
    private final NetworkTable limelightTable;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

  


}
