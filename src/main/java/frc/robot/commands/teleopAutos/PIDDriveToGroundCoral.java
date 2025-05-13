// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDDriveToGroundCoral extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  String m_camname;
  DoubleSupplier m_fwd;

  int tst;
  Constraints driveConstraints = new Constraints(3.5, 5);

  double coralAR = 11.875 / 4.5;

  double turnKP = .01;
  double strafeKP = .01;

  public PIDDriveToGroundCoral(SwerveSubsystem swerve, String camname, DoubleSupplier forwardSpeed) {
    m_swerve = swerve;
    m_camname = camname;
    m_fwd = forwardSpeed;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tst = 0;
    LimelightHelpers.setPipelineIndex(m_camname, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(m_camname)) {
      double x = LimelightHelpers.getTX(m_camname);
      double y = LimelightHelpers.getTY(m_camname);

      double[] t2d = LimelightHelpers.getT2DArray(m_camname);
      double shortSide = t2d[13];
      double longSide = t2d[12];
      double aspectrat = longSide / shortSide;
      double skew = t2d[16];
      double aspectfraction = aspectrat / coralAR;

      double angleDegrees = Units.radiansToDegrees(Math.acos(aspectfraction));

      Translation2d trans = new Translation2d(m_fwd.getAsDouble(), x * strafeKP);

      SD.sd2("GRN/DINangdeg", angleDegrees);
      SD.sd2("GRN/DINShort", shortSide);
      SD.sd2("GRN/DINLong", longSide);
      SD.sd2("GRN/DINasp", aspectfraction);
      SD.sd2("GRN/DINSkew", skew);

      // m_swerve.drive(trans, angleDegrees * turnKP, false, false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
