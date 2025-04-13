package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private SwerveSubsystem m_swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier angleCorrection;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

    public PIDController thetaController = new PIDController(1.5, 0, 0);

    public TeleopSwerve(
            SwerveSubsystem swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier angleCorrection) {
        m_swerve = swerve;
        addRequirements(m_swerve);

        this.angleCorrection = angleCorrection;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

    }

    @Override
    public void initialize() {
        thetaController.setTolerance(Rotation2d.fromDegrees(3).getRadians());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        Rotation2d target;
        double distanceToCoralStation = m_swerve.coralStationFinalTargetPose.getTranslation()
        .getDistance(m_swerve.getPose().getTranslation());
        if (distanceToCoralStation < 2) {
            target = m_swerve.coralStationFinalTargetPose.getRotation();
        } else {
            target = m_swerve.reefTargetPose.getRotation();
            target = target.rotateBy(new Rotation2d(Math.PI));
        }

        double translationVal = translationLimiter.calculate(
                -MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND));
        double strafeVal = strafeLimiter.calculate(
                -MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.DEADBAND));

        double rotationVal;

        if (angleCorrection.getAsBoolean()) {
            rotationVal = thetaController
                    .calculate(m_swerve.getPose().getRotation().getRadians(), target.getRadians());
        } else {
            rotationVal = //rotationLimiter.calculate(
                    -MathUtil.applyDeadband(rotationSup.getAsDouble(), OperatorConstants.RIGHT_X_DEADBAND);//);
            rotationVal = rotationVal * 1;
        }

        /* Drive */
        m_swerve.drive(new Translation2d(
                translationVal * m_swerve.swerveDrive.getMaximumChassisVelocity(),
                strafeVal * m_swerve.swerveDrive.getMaximumChassisVelocity()),
                rotationVal * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                true);

        SmartDashboard.putNumber("TransVal", translationVal);
        SmartDashboard.putNumber("StrafeVal", strafeVal);
        SmartDashboard.putNumber("RotVal", rotationVal);

    }
}