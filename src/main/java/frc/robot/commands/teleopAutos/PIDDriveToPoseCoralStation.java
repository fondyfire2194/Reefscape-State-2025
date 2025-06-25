// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PIDDriveToPoseCoralStation extends Command {
  private final SwerveSubsystem swerve;

  private final PIDController xController = new PIDController(2.8, 0, 0);
  private final PIDController yController = new PIDController(2.8, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);

  /** Creates a new PIDDriveToPose. */
  public PIDDriveToPoseCoralStation(SwerveSubsystem swerve) {
    this.swerve = swerve;

    xController.setTolerance(0.0125);
    yController.setTolerance(0.0125);

    // xController.setIZone(0.2);
    // xController.setIntegratorRange(-0.05, 0.05);
    // yController.setIZone(0.2);
    // yController.setIntegratorRange(-0.05, 0.05);

    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d target = swerve.coralStationFinalTargetPose;

    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d tl2d = new Translation2d(xController.calculate(swerve.getPose().getX()),
        yController.calculate(swerve.getPose().getY()));

    swerve.drive(
        tl2d,
        thetaController.calculate(swerve.getPose().getRotation().getRadians()),
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 swerve.setCoralStationFinalDist(swerve.getPoseToPoseDistance(swerve.coralStationFinalTargetPose, swerve.getPose()));
    swerve.setCoralStationFinalAngle(swerve.getPoseToPoseRotation(swerve.coralStationFinalTargetPose, swerve.getPose())
        .getDegrees());

    SmartDashboard.putNumber("Coral/FDist", swerve.getReefFinalDistance());
    SmartDashboard.putNumber("Coral/FAng", swerve.getReefFinalAngle());


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
