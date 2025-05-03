// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GISArmSim extends SubsystemBase implements AutoCloseable {

  private final SparkMaxSim gisarmMotorSim;

  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  public static final double kArmReduction = 200;
  public static final double kArmMass = 8.0; // Kilograms
  public static final double kArmLength = Units.inchesToMeters(30);
  public final GroundIntakeSubsystem m_gis;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  double al = 10;

  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", al, -90));

  private final MechanismLigament2d m_gisarm;

  /** Subsystem constructor. */
  public GISArmSim(GroundIntakeSubsystem gis) {
    m_gis = gis;
    gisarmMotorSim = new SparkMaxSim(m_gis.groundIntakeArmMotor, m_armGearbox);

    m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        m_gis.gearReduction,
        SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
        kArmLength,
        m_gis.minAngle,
        m_gis.maxAngle,
        true,
        m_gis.minAngle,
        0,
        0.0 // Add noise with a std-dev of 1 tick
    );

    m_gisarm = m_armPivot.append(
        new MechanismLigament2d(
            "GISArm",
            30,
            0,
            1,
            new Color8Bit(Color.kRed)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData(" GIS Arm Sim", m_mech2d);

    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }


  public void periodic(){

    SmartDashboard.putNumber("GISArmSim/SimAngle", m_gisarm.getAngle());
    SmartDashboard.putNumber("GISArmSim/EncoderAngle", m_gis.getMotorDegrees());

    SmartDashboard.putNumber("GISArmSim/SimEncoder", gisarmMotorSim.getPosition());
    SmartDashboard.putNumber("GISArmSim/APPO", gisarmMotorSim.getAppliedOutput() * 12);
    SmartDashboard.putNumber("GISArmSim/SimVel", gisarmMotorSim.getVelocity());

    SmartDashboard.putBoolean("GISArmSim/SimUpperLim", m_armSim.hasHitUpperLimit());
    SmartDashboard.putBoolean("GISArmSim/SimLowerLim", m_armSim.hasHitLowerLimit());
    SmartDashboard.putNumber("GISArmSim/SimAmps", m_armSim.getCurrentDrawAmps());


  }

  /** Update the simulation model. */
  public void simulationPeriodic() {

    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(gisarmMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    m_gisarm.setAngle(Units.radiansToDegrees(gisarmMotorSim.getPosition()));

   double vmps = gisarmMotorSim.getAppliedOutput();
    gisarmMotorSim.iterate(
      vmps * 60,
      RobotController.getBatteryVoltage(),
      0.02);
  }

  public void stop() {

  }

  @Override
  public void close() {

  }
}
