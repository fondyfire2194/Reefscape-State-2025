// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.io.FileFilter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class PositionHoldArmPID extends Command {
    private final ArmSubsystem arm;

    private PIDController pidController;
    private double kp = 25;
    private double ki = 0;
    private double kd;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxplusrate = 6;
    private double maxminusrate = 3;
    private boolean toggle;

    public double ffgain = .2;

    public PositionHoldArmPID(ArmSubsystem arm) {
        this.arm = arm;
        pidController = new PIDController(kp, ki, kd);
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        if (RobotBase.isSimulation())
            pidController.setP(10);
        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = arm.getAngleRadians();
        arm.setGoalRadians(temp);
        if (arm.showTelemetry) {
            SmartDashboard.putData(" Arm/Trap/controller", pidController);
        }
    }

    @Override
    public void execute() {

        toggle = !toggle;// split display for time purposes

        arm.nextSetpoint = arm.m_profile.calculate(.02, arm.currentSetpoint, arm.m_goal);

        double radpersec = pidController.calculate(arm.getAngleRadians(), arm.nextSetpoint.position);

        double nextVel = arm.nextSetpoint.velocity;

        double ksvrps = (arm.armKs) * Math.signum(nextVel) / arm.armKv;

        double kgrps = arm.armKg / arm.armKv;

        double accel = (arm.currentSetpoint.velocity - arm.nextSetpoint.velocity) * 50;

        double accelmps = (accel * arm.armKa) / arm.armKv;

        double velff = arm.nextSetpoint.velocity * ffgain;

        double radpersectotal = radpersec + velff + kgrps + ksvrps + accelmps;

        double radpersecclamped = MathUtil.clamp(radpersectotal, -maxminusrate, maxplusrate);

        arm.runAtVelocity(radpersecclamped);

        arm.currentSetpoint = arm.nextSetpoint;
        
        if (arm.showTelemetry) {
            if (toggle) {
                SmartDashboard.putNumber("Arm/Trap/goalpos", Units.radiansToDegrees(arm.m_goal.position));
                SmartDashboard.putNumber("Arm/Trap/currsetpos", Units.radiansToDegrees(arm.currentSetpoint.position));
                SmartDashboard.putNumber("Arm/Trap/currsetvel", Units.radiansToDegrees(arm.currentSetpoint.velocity));
                SmartDashboard.putNumber("Arm/Trap/setpos", Units.radiansToDegrees(arm.nextSetpoint.position));
            } else {
                SmartDashboard.putNumber("Arm/Trap/setvel", Units.radiansToDegrees(arm.nextSetpoint.velocity));
                SmartDashboard.putNumber("Arm/Trap/degpersec", Units.radiansToDegrees(radpersec));
                SmartDashboard.putNumber("Arm/Trap/poserror", Units.radiansToDegrees(pidController.getError()));
                SmartDashboard.putBoolean("Arm/Trap/atsetpoint", pidController.atSetpoint());
                SmartDashboard.putNumber("Arm/Trap/dpsclamped", Units.radiansToDegrees(radpersecclamped));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}