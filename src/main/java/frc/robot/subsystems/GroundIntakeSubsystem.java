package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANIDConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class GroundIntakeSubsystem extends SubsystemBase implements Logged {

    public final SparkMax groundIntakeArmMotor = new SparkMax(CANIDConstants.groundIntakeArmMotorID,
            MotorType.kBrushless);
    public final SparkMax groundIntakeRollerMotor = new SparkMax(CANIDConstants.groundIntakeRollerMotorID,
            MotorType.kBrushless);

    private SparkClosedLoopController groundIntakeArmClosedLoopController = groundIntakeArmMotor
            .getClosedLoopController();

    @Log(key = "alert warning")
    private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
    @Log(key = "alert error")
    private Alert allErrors = new Alert("AllErrors", AlertType.kError);
    @Log(key = "alert sticky fault")
    private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

    SparkMaxConfig groundintakeArmConfig;
    SparkMaxConfig groundintakerollerConfig;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public double angleToleranceDeg = 10;

    public boolean groundsetOnce;

    public double gearReduction = 125;// 100.;
    double radperencderrev = (Math.PI * 2) / gearReduction;

    double posConvFactor = radperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxmotorrps = 11000 / 60;// 5700 / 60;

    double maxdegpersec = radperencderrev * maxmotorrps;//

    public double groundIntakeArmKp = 0.5;// 0.075;

    public final double groundIntakeArmKi = 0;
    public final double groundIntakeArmKd = 0;

    double TRAJECTORY_VEL = 200;
    double TRAJECTORY_ACCEL = 200;

    public final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            TRAJECTORY_VEL, TRAJECTORY_ACCEL));

    @Log.NT(key = "goal")
    public TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    @Log.NT(key = "setpoint")
    public TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

    /**
     * Angles are set so that 90 degrees is with the groundintake balanced over
     * center
     * This means kg will act equally on both sides of top center
     * 
     */

    public final double minAngle = Units.degreesToRadians(90);
    public final double maxAngle = Units.degreesToRadians(240);
    public final double homeAngle = Units.degreesToRadians(100);

    public final double pickupAngle = Units.degreesToRadians(230);
    public final double deliverAngle = Units.degreesToRadians(170);
    public final double pickupSpeed = .8;

    public final double groundintakerollerKp = .00002; // P gains caused oscilliation
    public final double groundintakerollerKi = 0.0;
    public final double groundintakerollerKd = 0.00;
    public final double groundintakerollerKFF = .9 / 11000;

    public final double maxRollerMotorRPM = 5700;

    public IdleMode currentMode = IdleMode.kBrake;
    private int giInOutCoralAmps;
    public boolean coralAtGroundIntake;
    public boolean simCoralAtGroundIntake;
    public double noCoralAtIntakeTime = 15;
    private double deliverSpeed = .5;
    @Log
    public boolean groundCoralMode;


    public GroundIntakeSubsystem() {

        if (RobotBase.isSimulation())
            noCoralAtIntakeTime = 2;
        groundintakeArmConfig = new SparkMaxConfig();

        groundintakeArmConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);

        groundintakeArmConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        groundintakeArmConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(groundIntakeArmKp)
                .outputRange(-0.4, 0.4);

        groundintakeArmConfig.softLimit.forwardSoftLimit(maxAngle)
                .reverseSoftLimit(minAngle)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        groundintakeArmConfig.signals.primaryEncoderPositionPeriodMs(20);

        groundIntakeArmMotor.configure(groundintakeArmConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        groundintakerollerConfig = new SparkMaxConfig();

        groundintakerollerConfig
                .inverted(true)
                .smartCurrentLimit(giInOutCoralAmps)
                .idleMode(IdleMode.kBrake);

        groundintakerollerConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

        groundintakerollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .velocityFF(groundintakerollerKFF)
                .pid(groundintakerollerKp, groundintakerollerKi, groundintakerollerKd);

        groundintakerollerConfig.signals.primaryEncoderPositionPeriodMs(20);

        groundIntakeRollerMotor.configure(groundintakerollerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        groundIntakeArmMotor.getEncoder().setPosition(minAngle);

        m_goal.position = minAngle;

        SmartDashboard.putNumber("GIS/Values/maxdegpersec", maxdegpersec);
        SmartDashboard.putNumber("GIS/Values/inperencrev", radperencderrev);

    }

    public boolean getActiveFault() {
        return groundIntakeArmMotor.hasActiveFault();
    }

    public boolean getStickyFault() {
        return groundIntakeArmMotor.hasStickyFault();
    }

    public boolean getWarnings() {
        return groundIntakeArmMotor.hasActiveWarning();
    }

    public void positionGroundIntakeArm() {
        // Send setpoint to spark max controller
        nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

        groundIntakeArmClosedLoopController.setReference(
                nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        currentSetpoint = nextSetpoint;
    }

    public Command positionGroundIntakeArmCommand() {
        return Commands.run(() -> positionGroundIntakeArm(), this);
    }

    public void setMotorToCoast() {
        currentMode = IdleMode.kCoast;
        groundintakeArmConfig.idleMode(IdleMode.kCoast);
        groundIntakeArmMotor.configure(groundintakeArmConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void setMotorToBrake() {
        currentMode = IdleMode.kBrake;
        groundintakeArmConfig.idleMode(IdleMode.kBrake);
        groundIntakeArmMotor.configure(groundintakeArmConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Log.NT(key = "motor degrees")
    public double getMotorDegrees() {
        return Units.radiansToDegrees(groundIntakeArmMotor.getEncoder().getPosition());
    }

    public Command goHome() {
        return Commands.runOnce(() -> m_goal.position = minAngle);
    }

    public Command goReefLevel1() {
        return Commands.runOnce(() -> m_goal.position = deliverAngle);
    }

    public Command goPickup() {
        return Commands.runOnce(() -> m_goal.position = pickupAngle);
    }

    public Command jogGroundIntakeArmCommand(DoubleSupplier speed) {

        return new FunctionalCommand(
                () -> {
                }, // init
                () -> {
                    if (getMotorDegrees() <= maxAngle && speed.getAsDouble() > 0)
                        groundIntakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
                    else if (getMotorDegrees() >= minAngle && speed.getAsDouble() < 0)
                        groundIntakeArmMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage());
                    else
                        groundIntakeArmMotor.set(0);

                }, // execute
                (interrupted) -> groundIntakeArmMotor.set(0), // end
                () -> false, // isFinished
                this);// requirements
    }

    public Command stop() {
        return Commands.runOnce(() -> groundIntakeArmMotor.stopMotor());
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        allWarnings.set(getWarnings());
        allErrors.set(getActiveFault());
        allStickyFaults.set(getStickyFault());

        atUpperLimit = getArmAnglerads() > maxAngle;
        atLowerLimit = getArmAnglerads() < minAngle;
        SmartDashboard.putNumber("PIM/posdeg", Units.radiansToDegrees(groundIntakeArmMotor.getEncoder().getPosition()));
        // SmartDashboard.putBoolean("PIM/atpos", groundintakeAtStartPosition());
        SmartDashboard.putNumber("PIM/degpersec",
                Units.radiansToDegrees(groundIntakeArmMotor.getEncoder().getVelocity()));
        SmartDashboard.putNumber("PIM/volts",
                groundIntakeArmMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("PIM/amps", getArmAmps());

    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoder(double val) {
        groundIntakeArmMotor.getEncoder().setPosition(val);
    }

    public void setTolerance(double toleranceDeg) {
        angleToleranceDeg = toleranceDeg;
    }

    public Command setArmGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> m_goal.position = targetDegrees);
    }

    @Log(key = "anglerads")
    public double getArmAnglerads() {
        return groundIntakeArmMotor.getEncoder().getPosition();

    }

    public double getArmRadsPerSec() {
        return groundIntakeArmMotor.getEncoder().getVelocity();
    }

    @Log.NT(key = "groundintake degrees per sec")
    public double getArmDegreesPerSec() {
        return Units.radiansToDegrees(groundIntakeArmMotor.getEncoder().getVelocity());
    }

    public boolean onArmPlusSoftwareLimit() {
        return groundIntakeArmMotor.getEncoder().getPosition() >= groundIntakeArmMotor.configAccessor.softLimit
                .getForwardSoftLimit();
    }

    public boolean onArmMinusSoftwareLimit() {
        return groundIntakeArmMotor.getEncoder().getPosition() <= groundIntakeArmMotor.configAccessor.softLimit
                .getReverseSoftLimit();
    }

    @Log(key = "on limit")
    public boolean onLimit() {
        return onArmPlusSoftwareLimit() || onArmMinusSoftwareLimit();
    }

    public void stopArmMotor() {
        groundIntakeArmMotor.setVoltage(0);
    }

    public double getArmAmps() {
        return groundIntakeArmMotor.getOutputCurrent();
    }

    public boolean armIsBraked() {
        return groundIntakeArmMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getArmSoftwareLimitsEnabled() {
        return groundIntakeArmMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || groundIntakeArmMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return groundIntakeArmMotor.hasStickyFault() || groundIntakeRollerMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> groundIntakeArmMotor.clearFaults());
    }

    public Command jogGroundIntakeRollerCommand(DoubleSupplier speed) {
        return Commands
                .run(() -> groundIntakeRollerMotor
                        .setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()),
                        this);
    }

    @Log(key = "girolerrpm")
    public double getRollerRPM() {
        if (RobotBase.isReal())
            return groundIntakeRollerMotor.getEncoder().getVelocity();
        else
            return groundIntakeRollerMotor.get() * maxRollerMotorRPM;
    }

    public Command deliverCoralCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> groundIntakeRollerMotor.set(deliverSpeed)),
                new WaitCommand(0.25),
                Commands.runOnce(() -> simCoralAtGroundIntake = false),
                Commands.runOnce(() -> groundIntakeRollerMotor.stopMotor()));
    }
}
