// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class LimelightTagsUpdate {

    private final SwerveSubsystem m_swerve;
    private final CameraConstants.CameraValues m_cam;
    private boolean m_useMegaTag2 = false;
    boolean rejectUpdate;
    StructPublisher<Pose2d> mt2PosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TagUpdate/MT2Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> mt1PosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TagUpdate/MT1Pose", Pose2d.struct).publish();

    public LimelightTagsUpdate(CameraConstants.CameraValues cam, SwerveSubsystem swerve) {
        m_cam = cam;
        m_swerve = swerve;

    }

    public void setUseMegatag2(boolean on) {
        m_useMegaTag2 = on;
    }

    public void setLLRobotorientation() {
        SmartDashboard.putNumber("TagUpdate/LLGyroRead", m_swerve.getPose().getRotation().getDegrees());
        LimelightHelpers.SetRobotOrientation(m_cam.camname,
                m_swerve.getPose().getRotation().getDegrees(),
                m_swerve.getGyroRate(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
    }

    public void execute() {
        rejectUpdate = true;
        if (m_cam.isActive && LimelightHelpers.getTV(m_cam.camname)) {
            setLLRobotorientation();
            if (m_useMegaTag2) {
                setLLRobotorientation();
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
                m_swerve.distanceLimelightToEstimator = mt2.rawFiducials[0].distToCamera;

                rejectUpdate = mt2.tagCount == 0
                        || Math.abs(m_swerve.getGyroRate()) > 720
                        || (mt2.tagCount == 1 && mt2.rawFiducials[0].ambiguity > .7)
                        || mt2.rawFiducials[0].distToCamera > 4;

                SmartDashboard.putBoolean("TagUpdate/RejectUpdateMT2" + m_cam.camname, rejectUpdate);

                mt2PosePublisher.set(mt2.pose);

                if (!rejectUpdate) {
                    double standard_devs = mt2.rawFiducials[0].distToCamera / 2;// 0.5
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(
                            VecBuilder.fill(standard_devs,
                                    standard_devs, 9999999));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                }

            } else {

                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);

                rejectUpdate = mt1.tagCount == 0
                        || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 &&
                                mt1.rawFiducials[0].ambiguity > .7
                                && mt1.rawFiducials[0].distToCamera > 5;

                SmartDashboard.putBoolean("TagUpdate/RejectUpdateMT1" + m_cam.camname, rejectUpdate);
                mt2PosePublisher.set(mt1.pose);
                // SmartDashboard.putNumber("LLMT1 X", mt1.pose.getTranslation().getX());
                // SmartDashboard.putNumber("LLMT1 Y", mt1.pose.getTranslation().getY());

                if (!rejectUpdate) {
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,
                            .7, 1));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            }
        }
    }

    // private boolean inField(Pose2d pose) {
    // boolean inLength = pose.getX() > 0 && pose.getX() <
    // FieldConstants.FIELD_LENGTH;
    // boolean inWidth = pose.getY() > 0 && pose.getX() <
    // FieldConstants.FIELD_WIDTH;

    // return inLength && inWidth;
    // }
}
