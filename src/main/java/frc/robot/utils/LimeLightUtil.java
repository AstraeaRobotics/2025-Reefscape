// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class LimeLightUtil {
    
    NetworkTable limelight;

    public static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
   
    public void setPipeline(double pipelineID) {
        getTable().getEntry("pipeline").setNumber(pipelineID);
    }
   
    public static double getTx() {
        return getTable().getEntry("limelight").getDouble(0);
    }

    public static double getTy() {
        return getTable().getEntry("ty").getDouble(0);
    }

    public static double getTa() {
        return getTable().getEntry("ta").getDouble(0);
    }

    public static boolean getTv() {
        return getTable().getEntry("tv").getBoolean( false);
    }

    public static int getTagID() {
        double tagId = getTable().getEntry("tid").getDouble(-1);
        return (int) tagId;
    }

    public boolean atTarget() {
        return getTable().getEntry("tv").getDouble(0.0) == 1.0;
    }
   


}


