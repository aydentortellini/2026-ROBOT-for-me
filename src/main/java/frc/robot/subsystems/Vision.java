// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.TurretHelpers;

/** Subsystem for vision (Limelight) – targets and robot pose. */
public class Vision extends SubsystemBase {

  private final NetworkTable leftLimelight;
  private final NetworkTable rightLimelight;
  private final NetworkTable turretLimelight;
  private boolean turretCanSeeTags = false;
  final List<Integer> blueTagIds = Arrays.asList(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  final List<Integer> redTagIds = Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  final List<Integer> tagIds;

  public Vision() {
    leftLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.LEFT_TABLE);
    rightLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.RIGHT_TABLE);
    turretLimelight =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.TURRET_TABLE);

    tagIds =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redTagIds : blueTagIds;
  }

  /**
   * Get the primary Limelight table (e.g. "limelight"). For dual cameras use
   * getTable(LEFT_TABLE) / getTable(RIGHT_TABLE) in logic.
   */
  public static NetworkTable getTable(String name) {
    return NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {}
  
  public void setRobotPose(String limelight, Swerve drivetrain) {
    if (limelight == null || limelight.isEmpty()) {
      return;
    }
    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(limelight);
    LimelightHelpers.PoseEstimate bestMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

    if (bestMeasurement != null) {
      Pose2d newPose = pose.toPose2d();

      LimelightHelpers.SetRobotOrientation(
          limelight, newPose.getRotation().getDegrees(), 0, drivetrain.getPigeon2().getPitch().getValueAsDouble(), 0, drivetrain.getPigeon2().getRoll().getValueAsDouble(), 0);
    }
  }

  public int getTagId(NetworkTable table) {
    return (int) table.getEntry("tid").getInteger(0);
  }

  public String getBestLimelight() {
    LimelightHelpers.PoseEstimate leftMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate rightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    LimelightHelpers.PoseEstimate turretMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");

    String bestLimelight = "";
    double bestArea = 0;
    
    SmartDashboard.putNumber("turretArea", turretMeasurement.avgTagArea);
    SmartDashboard.putNumber("leftArea", leftMeasurement.avgTagArea);
    SmartDashboard.putNumber("rightArea", rightMeasurement.avgTagArea);

    if (turretMeasurement != null && turretMeasurement.avgTagArea > bestArea) {
      turretCanSeeTags = true;
      bestLimelight = "limelight-turret";
      bestArea = turretMeasurement.avgTagArea;
      PowerRobotContainer.setData("bestLimelight", bestLimelight);
      return "limelight-turret";
    } else {
      turretCanSeeTags = false;
    }

    if (leftMeasurement != null && leftMeasurement.avgTagArea > bestArea) {
      bestLimelight = "limelight-left";
      bestArea = leftMeasurement.avgTagArea;
    }

    if (rightMeasurement != null && rightMeasurement.avgTagArea > bestArea) {
      bestLimelight = "limelight-right";
      bestArea = rightMeasurement.avgTagArea;
    }

    PowerRobotContainer.setData("bestLimelight", bestLimelight);

    return bestLimelight;
  }

  public boolean getTurretCanSeeTags () {
    return turretCanSeeTags;
  }
}
