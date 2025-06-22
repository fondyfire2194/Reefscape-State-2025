// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import monologue.Annotations.Log;
import monologue.Logged;

public class LimelightVision extends SubsystemBase implements Logged {
  /** Creates a new LimelightVision. */

  public boolean limelightExistsfront;

  boolean allcamsok;

  public boolean limelightExistsrear;

  public boolean inhibitFrontVision;
  public boolean inhibitRearVision;

  private int loopctr;

  public String frontname = VisionConstants.CameraConstants.frontCamera.camname;

  Optional<Pose3d> temp;

  final int[] autoTagFilter = new int[] { 10, 11, 6, 7, 8, 9, 21, 22, 17, 18, 19, 20 };

  Alert flCameraAlert = new Alert("FrontCameraProblem", AlertType.kWarning);
  Alert frCameraAlert = new Alert("FrontRightCameraProblem", AlertType.kError);
  Alert rearCameraAlert = new Alert("RearCameraProblem", AlertType.kInfo);
  @Log(key = "frontaccpose")
  public Pose2d frontAcceptedPose;
  @Log(key = "frontacccount")
  public int frontAcceptedCount;
  @Log(key = "frontrejectupdate")
  public boolean frontRejectUpdate;

  @Log(key = "rearacceptpose")
  public Pose2d rearAcceptedPose;
  public int rearAcceptedCount;

  StructPublisher<Pose2d> wpiBluePosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("LLV/WPIBluePose", Pose2d.struct).publish();

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(String camname) {
    return LimelightHelpers.getLimelightNTTable(camname).containsKey("tv");
  }

  public LimelightVision() {

    setCamToRobotOffset(VisionConstants.CameraConstants.frontCamera);

    setCamToRobotOffset(VisionConstants.CameraConstants.rearCamera);

  }

  public void setAprilTagFilter(String camname) {
    LimelightHelpers.SetFiducialIDFiltersOverride(camname, autoTagFilter);
  }


  @Override
  public void periodic() {

    if (RobotBase.isReal()) {
      limelightExistsfront = isLimelightConnected(CameraConstants.frontCamera.camname);
      limelightExistsrear = isLimelightConnected(CameraConstants.rearCamera.camname);
      CameraConstants.frontCamera.isActive = !inhibitFrontVision && limelightExistsfront;
      CameraConstants.rearCamera.isActive = !inhibitRearVision && limelightExistsrear;

      allcamsok = VisionConstants.CameraConstants.frontCamera.isUsed && limelightExistsfront
          && VisionConstants.CameraConstants.rearCamera.isUsed && limelightExistsrear;

      flCameraAlert.set(!CameraConstants.frontCamera.isActive);

      if (limelightExistsfront && LimelightHelpers.getTV(frontname))
        wpiBluePosePublisher.set(LimelightHelpers.getBotPose3d_wpiBlue(frontname).toPose2d());

    }
  }

  public void setCamToRobotOffset(VisionConstants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

}