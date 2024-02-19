// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.VisionStuffs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
    ta = 0;
    tx = 0;
    ty = 0;
    ts = 0;
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
public static double getTX(String limelightName){
  return getLimelightNTDouble(limelightName, "tx");
}
public static double getTY(String limelightName) {
  return getLimelightNTDouble(limelightName, "ty");
}
public static double getTA(String limelightName) {
  return getLimelightNTDouble(limelightName, "ta");
}

public static double getLatency_Pipeline(String limelightName) {
  return getLimelightNTDouble(limelightName, "tl");
}

public static double getLatency_Capture(String limelightName) {
  return getLimelightNTDouble(limelightName, "cl");
}

public static double getCurrentPipelineIndex(String limelightName) {
  return getLimelightNTDouble(limelightName, "getpipe");
}


public static String getJSONDump(String limelightName) {
  return getLimelightNTString(limelightName, "json");
}

public static boolean getTV(String limelightName) {
  return 1.0 == getLimelightNTDouble(limelightName, "tv");
}

//Get Pose Stuffs

private static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

  private static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
      }

      //Get Normal Bot Pose

      public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

      public static Pose2d getBotPose2d(String limelightName) {
        double[] result = getBotpose(limelightName);
        return toPose2D(result);
    }

       public static Pose3d getBotPose3d(String limelightName) {
      double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
      return toPose3D(poseArray);
  } 
  
//Red-Blue Pose
  public static double[] getBotPose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
}

public static double[] getBotPose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
}

public static Pose3d getBotPose3d_wpiRed(String limelightName) {
  double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  return toPose3D(poseArray);
}

public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
  double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  return toPose3D(poseArray);
}

public static Pose2d getBotPose2d_wpiRed(String limelightName) {

  double[] result = getBotPose_wpiRed(limelightName);
  return toPose2D(result);

}

public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

  double[] result = getBotPose_wpiBlue(limelightName);
  return toPose2D(result);
}

//Targetting Stuffs
public static double[] getBotPose_TargetSpace(String limelightName) {
  return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
}

public static double[] getCameraPose_TargetSpace(String limelightName) {
  return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
}

public static double[] getTargetPose_CameraSpace(String limelightName) {
  return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
}

public static double[] getTargetPose_RobotSpace(String limelightName) {
  return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
}

  // public static double getTX(String limelightName){
  //   return 
  // }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
