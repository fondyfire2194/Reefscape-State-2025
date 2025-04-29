// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationRobotConstants;

public class ElevatorArmSim extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
  // Standard classes for controlling our elevator

  // Simulation classes help us simulate what'shboard going on, including gravity.
  private final ElevatorSim m_elevatorSim;
  private final SparkMaxSim m_elevatorMotorSim;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);

  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 25, 0);

  private final MechanismLigament2d m_elevatorLig2d;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private final SingleJointedArmSim m_armSim;

  private final SparkMaxSim m_armMotorSim;

  private final MechanismLigament2d m_armLig2d;

  private MechanismLigament2d m_armLig2d_1;

  /** Subsystem constructor. */
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;

  private boolean showValues = true;

  public ElevatorArmSim(ElevatorSubsystem elevator, ArmSubsystem arm) {
    m_elevator = elevator;
    m_arm = arm;
    m_armMotorSim = new SparkMaxSim(m_arm.armMotor, m_armGearbox);
    m_elevatorMotorSim = new SparkMaxSim(m_elevator.leftMotor, m_elevatorGearbox);
    m_elevatorSim = new ElevatorSim(
        m_elevatorGearbox,
        m_elevator.kElevatorGearing,
        m_elevator.kCarriageMass,
        m_elevator.kElevatorDrumRadiusMeters,
        m_elevator.minElevatorHeight.in(Meters),
        m_elevator.maxElevatorHeight.in(Meters),
        true,
        m_elevator.minElevatorHeight.in(Meters),
        0.0,
        0.0);

    m_elevatorLig2d = m_mech2dRoot.append(
        new MechanismLigament2d("Elevator",
            m_elevator.minElevatorHeight.in(Meters) * SimulationRobotConstants.kPixelsPerMeter,
            90));

    m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        m_arm.gearReduction*m_arm.beltPulleyRatio,
        SingleJointedArmSim.estimateMOI(m_arm.armLength, m_arm.armMass),
        m_arm.armLength,
        m_arm.minAngle.in(Radians),
        m_arm.maxAngle.in(Radians),
        true,
        m_arm.minAngle.in(Radians),
        0.01,
        0.0 // Add noise with a std-dev of 1 tick
    );

    m_armLig2d = m_elevatorLig2d.append(
        new MechanismLigament2d(
            "Arm",
            m_arm.armLength * SimulationRobotConstants.kPixelsPerMeter,
            0));

    m_armLig2d_1 = m_elevatorLig2d.append(
        new MechanismLigament2d(
            "Arm1",
            m_arm.armLength * SimulationRobotConstants.kPixelsPerMeter / 2,
            0));

    elevator.resetPosition(0);
    arm.resetEncoder(0);
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  @Override
  public void periodic() {
    // Update mechanism2d
    m_elevatorLig2d.setLength(
        m_elevator.minElevatorHeight.in(Meters)
            + m_elevator.leftMotor.getEncoder().getPosition() * SimulationRobotConstants.kPixelsPerMeter);

    m_armLig2d.setAngle(
        ( // mirror the angles so they display in the correct direction

        180 + Units.radiansToDegrees(m_arm.armMotor.getEncoder().getPosition()))
    // subtract 90 degrees to account for the elevator
    );

    m_armLig2d_1.setAngle(
        ( // mirror the angles so they display in the correct direction

        Units.radiansToDegrees(m_armMotorSim.getPosition()))
    // subtract 90 degrees to account for the elevator
    );
    if (showValues) {
      SmartDashboard.putNumber("ArmSim/SimAngle", m_armLig2d.getAngle());
      SmartDashboard.putNumber("ArmSim/EncoderAngle", m_arm.getMotorDegrees());

      SmartDashboard.putNumber("ArmSim/SimEncoder", m_armMotorSim.getPosition());
      SmartDashboard.putNumber("ArmSim/APPO", m_armMotorSim.getAppliedOutput() * 12);
      SmartDashboard.putNumber("ArmSim/SimVel", m_armMotorSim.getVelocity());

      SmartDashboard.putBoolean("ArmSim/SimUpperLim", m_armSim.hasHitUpperLimit());
      SmartDashboard.putBoolean("ArmSim/SimLowerLim", m_armSim.hasHitLowerLimit());
      SmartDashboard.putNumber("ArmSim/SimAmps", m_armSim.getCurrentDrawAmps());
    }
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)

    if (showValues) {
      SmartDashboard.putNumber("ElevatorSim/SIMAPPO", m_elevatorMotorSim.getAppliedOutput());
      SmartDashboard.putNumber("ElevatorSim/sim velocity", m_elevatorSim.getVelocityMetersPerSecond());
      SmartDashboard.putNumber("ElevatorSim/sim height", m_elevatorSim.getPositionMeters());
      SmartDashboard.putBoolean("ElevatorSim/simupperlimit", m_elevatorSim.hasHitUpperLimit());
      SmartDashboard.putBoolean("ElevatorSim/simlowerlimit", m_elevatorSim.hasHitLowerLimit());
      SmartDashboard.putNumber("ElevatorSim/simAmps", m_elevatorSim.getCurrentDrawAmps());
    }
    m_elevatorSim.setInput(m_elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    // elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    double vmps = m_elevatorSim.getVelocityMetersPerSecond();

    // if (m_elevatorMotorSim.getAppliedOutput() == 0)
    // vmps = 0;

    // Iterate the elevator and arm SPARK simulations
    m_elevatorMotorSim.iterate(
        vmps,
        RobotController.getBatteryVoltage(),
        0.02);

    vmps = m_armMotorSim.getAppliedOutput();

    // if (m_armMotorSim.getAppliedOutput() == 0)
    // vmps = 0;

    m_armMotorSim.iterate(
        vmps * 60,
        RobotController.getBatteryVoltage(),
        0.02);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_elevator.elevatorCurrentTarget =
    // m_elevator.leftMotor.getEncoder().getPosition();
    m_elevator.leftMotor.setVoltage(0.0);
    m_arm.armMotor.setVoltage(0);
  }

  @Override
  public void close() {
    m_elevator.leftMotor.close();
    m_mech2d.close();
  }
}
