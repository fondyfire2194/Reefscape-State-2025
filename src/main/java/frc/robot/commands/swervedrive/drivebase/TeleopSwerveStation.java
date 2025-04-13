package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleopSwerveStation extends Command {
    private SwerveSubsystem m_swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

    public PIDController thetaController = new PIDController(0.8, 0, 0);

    public TeleopSwerveStation(
            SwerveSubsystem swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup) {
        m_swerve = swerve;
        addRequirements(m_swerve);

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

        // double distanceToCoralStation = m_swerve.coralStationFinalTargetPose.getTranslation()
        // .getDistance(m_swerve.getPose().getTranslation());
        Rotation2d target = m_swerve.coralStationFinalTargetPose.getRotation();
        

        double translationVal = translationLimiter.calculate(
                -MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND));
        double strafeVal = strafeLimiter.calculate(
                -MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.DEADBAND));


        double rotationVal = thetaController
                    .calculate(m_swerve.getPose().getRotation().getRadians(), target.getRadians());
        

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