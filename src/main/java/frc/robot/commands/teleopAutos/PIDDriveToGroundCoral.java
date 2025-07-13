// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDDriveToGroundCoral extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  CommandXboxController m_controller;
  String m_camname;
  DoubleSupplier m_fwd;
  public boolean showTelemetry;
  private double imageWidth = 640;
  MedianFilter topLeftXFilter = new MedianFilter(5);
  MedianFilter topRightXFilter = new MedianFilter(5);
  MedianFilter ARFilter = new MedianFilter(5);

  double corners[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  int topLeftX = 0;
  int topRightX = 4;

  private double topLeftCornerXFiltered;

  private double topRightCornerXFiltered;

  Constraints driveConstraints = new Constraints(3.5, 5);

  double turnKP = .01;
  double strafeKP = .0075;
  double strafemax = .4;
  private double xerror;
  private double rot;
  private double AR;
  private boolean insideAR;
  private int ARctr;
  private boolean initalSign;
  private double sign;
  
    public PIDDriveToGroundCoral(SwerveSubsystem swerve, String camname, CommandXboxController controller) {
      m_swerve = swerve;
      m_camname = camname;
      m_controller = controller;
      addRequirements(m_swerve);
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      LimelightHelpers.setPipelineIndex(m_camname, 2);
      insideAR = false;
      ARctr = 0;
      initalSign = false;
      sign = 1;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      xerror = 0;
      rot = 0;
      if (LimelightHelpers.getTV(m_camname)) {
        rot = LimelightHelpers.getTX(m_camname);
        double y = LimelightHelpers.getTY(m_camname);
        corners = LimelightHelpers.getLimelightNTDoubleArray("limelight-rear", "tcornxy");
        if (corners.length == 8) {
          double topRightCornerX = corners[topRightX];
          double topLeftCornerX = corners[topLeftX];
          topLeftCornerXFiltered = topLeftXFilter.calculate(topLeftCornerX);
          topRightCornerXFiltered = topRightXFilter.calculate(topRightCornerX);
  
          double centerX = getCenterX(topLeftCornerXFiltered, topRightCornerXFiltered);
          double target = imageWidth / 2;
          xerror = getOffsetFromTarget(centerX, target);
          double width = topRightCornerXFiltered - topLeftCornerXFiltered;
  
          double height = corners[5] - corners[1];
          AR = ARFilter.calculate(width / height);
          if (showTelemetry) {
            SD.sd2("TopRightXF", topRightCornerXFiltered);
            SD.sd2("TopLeftXF", topLeftCornerXFiltered);
  
            SD.sd2("XCenter", centerX);
            SD.sd2("XError", xerror);
            SD.sd2("TX", rot);
  
            SD.sd2("Width", width);
            SD.sd2("Height", height);
            SD.sd2("AR", width / height);
  
            // for (int j = 0; j < 8; j++) {
  
            // SD.sd("Corners " + String.valueOf(j), corners[j]);
            // }
          }
        }
        double turn = 0;
        double strafe = 0;
        
        if (ARctr < 20 && AR < 2.25) {
          if (!initalSign) {
            sign = Math.signum(rot);
          initalSign = true;
        }
        turn = (2.45 - AR) * sign;
        
      } else {
        ARctr++;
      }
      
      if (ARctr > 20) {
        strafe = -xerror * strafeKP;
        turn = 0;
      }
      SmartDashboard.putBoolean("INAR", insideAR);
      strafe = MathUtil.clamp(strafe, -strafemax, strafemax);
      Translation2d trans = new Translation2d(-m_controller.getLeftY(), strafe);
      if (Math.abs(strafe) < 0.08) {
         trans = new Translation2d(-0.5, 0);
      } 
      m_swerve.drive(trans, -turn, false, false);

    } else {
      Translation2d trans = new Translation2d(-m_controller.getLeftY(), m_controller.getLeftX());
      // m_swerve.drive(trans, m_controller.getRightX(), false, false);
    }
  }

  public static double getCenterX(double x1, double x2) {
    return (x1 + x2) / 2.0;
  }

  public static double getOffsetFromTarget(double centerX, double target) {

    return target - centerX; // Positive = left, Negative = right
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
