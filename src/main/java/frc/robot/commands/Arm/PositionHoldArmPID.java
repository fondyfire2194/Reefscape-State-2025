// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.SD;

public class PositionHoldArmPID extends Command {
    private final ArmSubsystem arm;

    private PIDController pidController;
    private double kp = 15;
    private double ki = 0;
    private double kd;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxplusrate = 6;
    private double maxminusrate = 6;

    private boolean toggle;

    private double ffGain = .5;

    public PositionHoldArmPID(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation())
            kp = 10;

        pidController = new PIDController(kp, ki, kd);
        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = arm.getAngleRadians();
        arm.setGoalRadians(temp);
        if (arm.showTelemetry)
            SmartDashboard.putData(" Arm/PID/controller", pidController);
    }

    @Override
    public void execute() {

        toggle = !toggle;

        arm.nextSetpoint = arm.m_profile.calculate(.02, arm.currentSetpoint, arm.m_goal);

        double radpersec = pidController.calculate(arm.getAngleRadians(), arm.nextSetpoint.position);

        double armVelFF = arm.nextSetpoint.velocity * ffGain;

        radpersec += armVelFF;

        if (arm.showTelemetry) {
            if (toggle) {
                SD.sd2("Arm/PID/goalpos", Units.radiansToDegrees(arm.m_goal.position));
                SD.sd2("Arm/PID/currsetpos", Units.radiansToDegrees(arm.currentSetpoint.position));
                SD.sd2("Arm/PID/currsetvel", Units.radiansToDegrees(arm.currentSetpoint.velocity));
                SD.sd2("Arm/PID/setpos", Units.radiansToDegrees(arm.nextSetpoint.position));
            } else {
                SD.sd2("Arm/PID/setvel", Units.radiansToDegrees(arm.nextSetpoint.velocity));
                SD.sd2("Arm/PID/degpersec", Units.radiansToDegrees(radpersec));
                SD.sd2("Arm/PID/poserror", Units.radiansToDegrees(pidController.getError()));
                SmartDashboard.putBoolean("Arm/PID/atSetpoint", pidController.atSetpoint());
            }
        }
        radpersec = MathUtil.clamp(radpersec, -maxminusrate, maxplusrate);

        if (arm.showTelemetry)
            SD.sd2("Arm/PID/dpsclamped", Units.radiansToDegrees(radpersec));

        double volts = RobotController.getBatteryVoltage() * radpersec / arm.maxradpersec;

        arm.armMotor.setVoltage(volts);

        // arm.runAtVelocity(radpersec);

        arm.currentSetpoint = arm.nextSetpoint;

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}