// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JogElevator extends Command {
    /** Creates a new JogArm. */
    private final CommandXboxController gamepad;
    private final ElevatorSubsystem elevator;
    private final double deadband = .00;

    public JogElevator(ElevatorSubsystem elevator, CommandXboxController gamepad) {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        // gamepad.rumble(250);
    }

    @Override
    public void execute() {
        double stickValue = 0;
        if (RobotBase.isReal())
            stickValue = -gamepad.getRightY() / 6;
        else
            stickValue = -gamepad.getRawAxis(5);
        if (Math.abs(stickValue) < deadband)
            stickValue = 0;

        double leftPower = stickValue;

        boolean overrideLimits = gamepad.start().getAsBoolean();

        if (overrideLimits ||
                (leftPower > 0 && !elevator.atUpperLimit)
                || leftPower < 0 && !elevator.atLowerLimit) {
            elevator.leftMotor.set(leftPower);

        } else {
            elevator.leftMotor.set(0);

        }
        SmartDashboard.putNumber("Elevator/jogttes", leftPower);
        SmartDashboard.putNumber("Elevator/posn", elevator.getLeftPositionMeters());

        double volts = elevator.leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

        SmartDashboard.putNumber("Elevator/JOGVOLTS", volts);
        double vpermps = volts / elevator.getLeftVelocityMetersPerSecond();
        SmartDashboard.putNumber("Elevator/VPERMPS", vpermps);

        elevator.setGoalMeters(elevator.getLeftPositionMeters());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setGoalMeters(elevator.getLeftPositionMeters());
        elevator.leftMotor.setVoltage(0);
        // gamepad.rumble(250);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
