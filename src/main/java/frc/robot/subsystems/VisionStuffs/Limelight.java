// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.VisionStuffs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private static Limelight m_Instance;


  private double[] cameraPose_targetSpace;

  private double[] robotPose_FieldSpace;

  private  double[] robotPose_TargetSpace;

  private double[] targetPose_CameraSpace;

  private double[] targetPose_RobotSpace;

  private double[] cameraPose_TargetSpace;

  public double ta;

  public double tx;

  public double tx_pixels;

  public double ty;

  public double ty_pixels;

  public double ts;

  public String name;


  public Limelight() {
    cameraPose_TargetSpace = new double[6];
    robotPose_FieldSpace = new double[6];
    robotPose_TargetSpace = new double[6];
    targetPose_CameraSpace = new double[6];
    targetPose_RobotSpace = new double[6];
  }

  static final String sanitizeName(String name) {
    if (name == "" || name == null) {
        return "limelight";
    }
    return name;
}

  public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }
  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
      return getLimelightNTTable(tableName).getEntry(entryName);
  }
  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
}
  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
}
public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
  getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
}
public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
  return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
}
public static String getLimelightNTString(String tableName, String entryName) {
  return getLimelightNTTableEntry(tableName, entryName).getString("");
}










  // public static double getTX(String limelightName){
  //   return 
  // }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
