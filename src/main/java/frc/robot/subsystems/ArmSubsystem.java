package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIDConstants;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import monologue.Annotations.Log;
import monologue.Logged;

public class ArmSubsystem extends SubsystemBase implements Logged {

    public final SparkMax armMotor = new SparkMax(CANIDConstants.armMotorID, MotorType.kBrushless);

    private SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();

    @Log(key = "alert warning")
    private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
    @Log(key = "alert error")
    private Alert allErrors = new Alert("AllErrors", AlertType.kError);
    @Log(key = "alert sticky fault")
    private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

    public ArmFeedforward armfeedforward;

    SparkMaxConfig armConfig;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public Angle angleToleranceRads = Radians.of(Units.degreesToRadians(.5));

    public boolean presetOnce;

    public double gearReduction = 20.;
    public double beltPulleyRatio = 1.5;
    public double armLength = Units.inchesToMeters(20);
    public double armMass = Units.lbsToKilograms(4.3);
    double radperencderrev = (2 * Math.PI) / (beltPulleyRatio * gearReduction);

    double posConvFactor = radperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxmotorrps = 5700 / 60;

    public double maxradpersec = radperencderrev * maxmotorrps;//

    double maxdegrespersec = Units.radiansToDegrees(maxradpersec);

    /*
     * ( (value that goes up) - (value that goes down) )/ 2 = ks .4up .04 down
     * ( (value that goes up) + (value that goes down) )/2 = kg
     */

    public final double armKg = 0.3;
    public final double armKs = 0.18;
    public final double armKv = 12 / maxradpersec;
    public final double armKa = 0.05;

    public double armKp = 0.03;

    public final double armKi = 0.;
    public final double armKd = 0;

    /**
     * Angles are set so that 90 degrees is with the arm balanced over center
     * This means kg will act equally on both sides of top center
     * 
     */

    public final Angle armStartupOffset = Degrees.of(134);
    public final Angle minAngle = Degrees.of(-10);
    public final Angle maxAngle = armStartupOffset;

    public double armClearAngleDeg = 104;

    double TRAJECTORY_VEL = 2 * Math.PI;
    double TRAJECTORY_ACCEL = 6 * Math.PI;

    public final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            TRAJECTORY_VEL, TRAJECTORY_ACCEL));
    @Log(key = "goal")
    public TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    public TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

    private int inPositionCtr;

    public double targetRadians;

    private double armff;

    public boolean showTelemetry = true;

    public ArmSubsystem() {

        if (showTelemetry) {

            SmartDashboard.putNumber("Arm/Values/maxdegpersec", maxdegrespersec);
            SmartDashboard.putNumber("Arm/Values/poscf", posConvFactor);
            SmartDashboard.putNumber("Arm/Values/maxradpersec", maxradpersec);
            SmartDashboard.putNumber("Arm/Values/kv", armKv);
        }
        armConfig = new SparkMaxConfig();

        armConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        armConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(armKp)
                .outputRange(-1, 1)

                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1 / maxradpersec, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        armConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        armConfig.softLimit.forwardSoftLimit(maxAngle.in(Radians))
                .reverseSoftLimit(minAngle.in(Radians))
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        armConfig.signals.primaryEncoderPositionPeriodMs(10);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armfeedforward = new ArmFeedforward(armKa, armKg, armKv);

        armMotor.getEncoder().setPosition(armStartupOffset.in(Radians));

        setGoalRadians(armStartupOffset.in(Radians));

        SmartDashboard.putNumber("Arm/RealEncoder", Units.radiansToDegrees(armMotor.getEncoder().getPosition()));
        if (RobotBase.isSimulation())
            armKp = 5;
    }

    public boolean getActiveFault() {
        return armMotor.hasActiveFault();
    }

    public boolean getStickyFault() {
        return armMotor.hasStickyFault();
    }

    public boolean getWarnings() {
        return armMotor.hasActiveWarning();
    }

    public void disableSoftLimits() {
        armConfig.softLimit.forwardSoftLimitEnabled(false);
        armConfig.softLimit.reverseSoftLimitEnabled(false);
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void enableSoftLimits() {
        armConfig.softLimit.forwardSoftLimitEnabled(true);
        armConfig.softLimit.reverseSoftLimitEnabled(true);
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        allWarnings.set(getWarnings());
        allErrors.set(getActiveFault());
        allStickyFaults.set(getStickyFault());

        atUpperLimit = getAngle().gte(maxAngle);
        atLowerLimit = getAngle().lte(minAngle);
        // SmartDashboard.putNumber("Arm/pos",
        // Units.radiansToDegrees(armMotor.getEncoder().getPosition()));
        // SmartDashboard.putNumber("Arm/vel",
        // Units.radiansToDegrees(armMotor.getEncoder().getVelocity()));
        // SmartDashboard.putNumber("Arm/volts", armMotor.getAppliedOutput() *
        // RobotController.getBatteryVoltage());

    }

    @Override
    public void simulationPeriodic() {

    }

    // public boolean atGoal() {
    // return Math.abs( - getAngle().in(Radians)) < .01;
    // }

    public void resetEncoder(double val) {
        armMotor.getEncoder().setPosition(val);
    }

    public void position() {
        if (inPositionCtr != 3)
            inPositionCtr++;
        // Send setpoint to spark max controller
        nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

        // SmartDashboard.putNumber("Arm/setpos",
        // Units.radiansToDegrees(nextSetpoint.position));
        // SmartDashboard.putNumber("Arm/setvel",
        // Units.radiansToDegrees(nextSetpoint.velocity));

        armff = armfeedforward.calculate(getAngle().in(Radians), nextSetpoint.velocity);

        double accel = (nextSetpoint.velocity - currentSetpoint.velocity) * 50;

        double accelV = accel * armKa;

        armff = armff + accelV;

        currentSetpoint = nextSetpoint;

        // SmartDashboard.putNumber("Arm/ff", armff);
        // SmartDashboard.putNumber("Arm/poserror", m_goal.position -
        // armMotor.getEncoder().getPosition());

        armClosedLoopController.setReference(
                nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, armff,
                ArbFFUnits.kVoltage);

    }

    public void setTolerance(Angle toleranceRads) {
        angleToleranceRads = toleranceRads;
    }

    public void setGoalRadians(double targetRads) {
        m_goal.position = targetRads;
        currentSetpoint.position = getAngle().in(Radians);
        targetRadians = targetRads;
        SmartDashboard.putNumber("Arm/targetdeg", Units.radiansToDegrees(targetRads));
        // inPositionCtr = 0;
    }

    @Log.NT(key = "goal degrees")
    public void setGoalDegrees(double targetDegrees) {
        double targetRads = Units.degreesToRadians(targetDegrees);
        setGoalRadians(targetRads);
    }

    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> setGoalDegrees(targetDegrees));
    }

    public void runAtVelocity(double radiansPerSecond) {
        armClosedLoopController.setReference(radiansPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public double getAngleRadians() {
        return armMotor.getEncoder().getPosition();
    }

    @Log.NT(key = "motor degrees")
    public double getMotorDegrees() {
        return Units.radiansToDegrees(armMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    @Log(key = "angle")
    public Angle getAngle() {
        double rawAngle = armMotor.getEncoder().getPosition();
        return Radians.of(rawAngle);
    }

    public double getRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    @Log.NT(key = "arm degrees per sec")
    public double getDegreesPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return armMotor.getEncoder().getPosition() >= armMotor.configAccessor.softLimit.getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return armMotor.getEncoder().getPosition() <= armMotor.configAccessor.softLimit.getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return armMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return armMotor.getReverseLimitSwitch().isPressed();
    }

    @Log(key = "on limit")
    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return armMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || armMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return armMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    public boolean checkArmClear() {
        return armMotor.getEncoder().getPosition() > ArmSetpoints.kokElevatorMove;
    }

}
