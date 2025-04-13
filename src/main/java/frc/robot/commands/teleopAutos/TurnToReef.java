// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnToReef extends Command {

  // Software PID turn constants
  // private final double kP = 0.15958;
  private final double kP = 2;
  private final double kI = 0.0;
  private final double kD = 0.0;


  private final double kMaxSpeed = Constants.MAX_SPEED;// veChassis.MaxAngularRate * 180.0 / Math.PI; // degrees per
                                                       // second
  private final double kMaxAngularRate = Constants.MAX_ANGULAR_VEL;
  private final double kMaxAccel = Constants.MAX_ANGULAR_ACCEL;//
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
  private final ProfiledPIDController ppc = new ProfiledPIDController(kP, kI, kD, constraints);

  private final double tolerance = Units.degreesToRadians(.5); // degrees of tolerance to end the command

  private final SwerveSubsystem m_swerve;

  /** Creates a new TurnToRelativeAngleTrapezoidProfile. */
  public TurnToReef(SwerveSubsystem swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ppc.enableContinuousInput(-Math.PI, Math.PI);
    double initialAngle = m_swerve.getPose().getRotation().getRadians();
    ppc.reset(initialAngle, 0); // set current angle and 0 angular velocity as current state
    ppc.setGoal(m_swerve.reefFinalTargetPose.getRotation().getRadians());
    ppc.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPose = m_swerve.getPose().getRotation().getRadians();
    double ff = ppc.calculate(currentPose);

    // ff = (ff >= 0) ? MathUtil.clamp(ff, minFeedForward * kMaxAngularRate,
    // kMaxAngularRate)
    // : MathUtil.clamp(ff, -kMaxAngularRate, -minFeedForward * kMaxAngularRate);

    ff *= kMaxAngularRate *.5;

    // SmartDashboard.putNumber("ROTATEFF", ff);
    // SmartDashboard.putNumber("ROTATEERR", ppc.getPositionError());
    // SmartDashboard.putBoolean("ROTATEATGOAL", ppc.atGoal());
    Translation2d t2d = new Translation2d(0, 0);
    m_swerve.drive(t2d, ff, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ppc.atGoal();// || counter > counterFinal;
  }
}
