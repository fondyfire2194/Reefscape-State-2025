// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.commands.Arm.PositionHoldArm;
import frc.robot.commands.Elevator.PositionHoldElevator;
import frc.robot.commands.Elevator.PositionHoldElevatorPID;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SD;
import monologue.Logged;
import monologue.Monologue;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements Logged {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private Timer oneShotTimer;

  Pose2d startingPoseAtBlueAlliance = new Pose2d();

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   * 
   * Radio SSID is Fondy25
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    Monologue.setupMonologue(m_robotContainer, "/Monologue", false, true);
    DriverStation.startDataLog(DataLogManager.getLog());

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
    if (RobotBase.isReal()) {
      URCL.start();
    }

    oneShotTimer = new Timer();
    oneShotTimer.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // setFileOnly is used to shut off NetworkTables broadcasting for most logging
    // calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    Monologue.setFileOnly(DriverStation.isFMSAttached());
    // This method needs to be called periodically, or no logging annotations will
    // process properly.
    Monologue.updateAll();

    if (oneShotTimer.hasElapsed(5)) {
      // Get the voltage going into the PDP, in Volts.
      // The PDP returns the voltage in increments of 0.05 Volts.
      double voltage = m_robotContainer.pdp.getVoltage();
      SD.sd2("PDP/Voltage", voltage);
      // Retrieves the temperature of the PDP, in degrees Celsius.
      double temperatureCelsius = m_robotContainer.pdp.getTemperature();
      SD.sd2("PDP/TemperatureF", temperatureCelsius);
 
      oneShotTimer.restart();
    }

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();

    m_robotContainer.drivebase.frontUpdate.setLLRobotorientation();
    m_robotContainer.drivebase.rearUpdate.setLLRobotorientation();

    m_robotContainer.llv.inhibitFrontVision = false;
    m_robotContainer.llv.inhibitRearVision = true;
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }

    String autname = m_robotContainer.autoChooser.getSelected().getName();

    SmartDashboard.putString("AUT/name", autname);

    if (DriverStation.isAutonomous() && !DriverStation.isEnabled() && !autname.contains("Instant")) {

      if (m_robotContainer.drivebase.isBlueAlliance())
        startingPoseAtBlueAlliance = new PathPlannerAuto(autname).getStartingPose();
      m_robotContainer.drivebase.startingPose = m_robotContainer.drivebase.isRedAlliance()
          ? FlippingUtil.flipFieldPose(startingPoseAtBlueAlliance)
          : startingPoseAtBlueAlliance;

      m_robotContainer.drivebase.startPoseDifferenceX = Units.metersToInches(
          m_robotContainer.drivebase.startingPose.getX()
              - m_robotContainer.drivebase.getPose().getX());
      m_robotContainer.drivebase.startPoseDifferenceY = Units.metersToInches(
          m_robotContainer.drivebase.startingPose.getY()
              - m_robotContainer.drivebase.getPose().getY());
      m_robotContainer.drivebase.startPoseDifferenceTheta = m_robotContainer.drivebase.startingPose.getRotation()
          .getDegrees()
          - m_robotContainer.drivebase.getPose().getRotation().getDegrees();

    }

    // m_robotContainer.drivebase.resetOdometry(startingPose);

    LimelightHelpers.SetRobotOrientation(CameraConstants.frontCamera.camname,
        m_robotContainer.drivebase.getPose().getRotation().getDegrees(),
        // m_swerve.getHeadingDegrees(),
        m_robotContainer.drivebase.getGyroRate(), 0, 0, 0, 0);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (m_robotContainer.drivebase.isBlueAlliance())
      LimelightHelpers.SetFiducialIDFiltersOverride(CameraConstants.frontCamera.camname, FieldConstants.blueReefTags);
    else
      LimelightHelpers.SetFiducialIDFiltersOverride(CameraConstants.frontCamera.camname, FieldConstants.redReefTags);

    m_robotContainer.setMotorBrake(true);

    m_robotContainer.preIn.preIntakeToStartCommand().schedule();

    new PositionHoldArm(m_robotContainer.arm).schedule();
    new PositionHoldElevatorPID(m_robotContainer.elevator).schedule();
    // m_robotContainer.preIn.preIntakeToStartCommand().schedule();

    m_robotContainer.drivebase.frontUpdate.setUseMegatag2(true);
    m_robotContainer.drivebase.rearUpdate.setUseMegatag2(true);

    m_robotContainer.llv.inhibitFrontVision = false;
    m_robotContainer.llv.inhibitRearVision = true;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("Autname", m_autonomousCommand.getName());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

    new PositionHoldArm(m_robotContainer.arm).schedule();
    new PositionHoldElevatorPID(m_robotContainer.elevator).schedule();

    m_robotContainer.setMotorBrake(true);

    m_robotContainer.configureCoDriverTeleopBindings();

    m_robotContainer.drivebase.frontUpdate.setUseMegatag2(true);
    m_robotContainer.drivebase.rearUpdate.setUseMegatag2(true);
    m_robotContainer.llv.inhibitFrontVision = false;
    m_robotContainer.llv.inhibitRearVision = true;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("MatchTime", Timer.getMatchTime());

    if (!DriverStation.isTeleopEnabled()) {
      double totalAmps = m_robotContainer.pdp.getTotalCurrent();
      SD.sd2("PDP/TotalAmps", totalAmps);
      // Get the total power of all channels.
      // Power is the bus voltage multiplied by the current with the units Watts.
      double totalPower = m_robotContainer.pdp.getTotalPower();
      SD.sd2("PDP/TotalPower", totalPower);

      // Get the total energy of all channels.
      // Energy is the power summed over time with units Joules.
      double totalEnergy = m_robotContainer.pdp.getTotalEnergy();
      SD.sd2("PDP/TotalEnergy", totalEnergy);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    new PositionHoldArm(m_robotContainer.arm).schedule();

    // new PositionHoldElevatorStateSpace(m_robotContainer.elevator).schedule();
    // new PositionHoldElevatorExponential(m_robotContainer.elevator).schedule();
    new PositionHoldElevator(m_robotContainer.elevator).schedule();
    // new PositionHoldElevatorPID(m_robotContainer.elevator).schedule();

    m_robotContainer.configureCoDriverTestBindings();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {

  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
