// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Gamepieces.DetectAlgaeWhileIntaking;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LedStrip;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;
        ElevatorSubsystem m_elevator;
        ArmSubsystem m_arm;
        GamepieceSubsystem m_gamepieces;
        AlgaeSubsystem m_algae;
        LimelightVision m_llv;
        static CommandXboxController m_dr;
        static CommandXboxController m_codr;

        LedStrip m_ls;

        public CommandFactory(SwerveSubsystem swerve, ElevatorSubsystem elevator, ArmSubsystem arm,
                        GamepieceSubsystem gamepieces, AlgaeSubsystem algae, LimelightVision llv, LedStrip ls, CommandXboxController dr,
                        CommandXboxController codr) {
                m_swerve = swerve;
                m_dr = dr;
                m_codr = codr;
                m_arm = arm;
                m_elevator = elevator;
                m_llv = llv;
                m_gamepieces = gamepieces;
                m_algae=algae;
                m_ls = ls;

        }

        public Command pickupAlgaeL2() {
                return Commands.parallel(setSetpointCommand(Setpoint.kAlgaePickUpL2),
                                new DetectAlgaeWhileIntaking(m_algae));
        }

        public Command pickupAlgaeL3() {
                return Commands.none();
        }

        public Command deliverAlgaeToBargeCommand(double delaySecs) {
                return Commands.parallel(
                                m_arm.setGoalDegreesCommand(ArmSetpoints.kBargeDeliver),
                                Commands.sequence(
                                                Commands.waitSeconds(delaySecs),
                                                m_algae.deliverAlgaeToBargeCommand()));

        }

        public static Command rumbleDriver(RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> m_dr.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> m_dr.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        }

        public static Command rumbleCoDriver(RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> m_codr.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> m_codr.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        }

        public boolean isRedAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red;
                } else {
                        return false;
                }
        }

        public boolean isBlueAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Blue;
                } else {
                        return false;
                }
        }

        public enum Setpoint {
                kTravel,
                kCoralStation,
                kLevel1,
                kLevel2,
                kLevel3,
                kLevel4,
                kProcessorDeliver,
                kAlgaeDeliverBarge,
                kAlgaePickUpL3,
                kAlgaePickUpL2;
        }

        public static final class ElevatorSetpoints {
                public static final int kHome = 0;
                public static final int kTravel = 10;
                public static final int kProcessorDeliver = 8;
                public static final int kCoralStation = 0;
                public static final int kLevel1 = 6;
                public static final int kLevel2 = 15;
                public static final int kLevel3 = 30;
                public static final double kLevel4 = 57.5;
                public static final int kBarge = 65;
                public static final int kLevelAlgaeL2 = 30;
                public static final int kLevelAlgaeL3 = 47;

        }

        public static final class ArmSetpoints {

                public static final int kokElevatorMove = 90;
                public static final int kTravel = 100;
                public static final int kProcessorDeliver = 15;
                public static final int kBargeDeliver = 85;
                public static final double kCoralStation = 132;
                public static final double kLevel1 = 97;
                public static final double kLevel2 = 97;
                public static final double kLevel3 = 97;
                public static final double kLevel4_1 = 103;
                public static final double kLevel4_2 = 85;
                public static final double kAlgaeIntake = 0;

        }

        public static final class CoralRPMSetpoints {
                public static final double kCoralIntakeMotorRPM = 2000;
                public static final double kGmepieceCoralIntakeRPM = 5000;
                public static final double kGamepieceReefPlaceL123 = 4000;
                public static final double kGamepieceReefPlaceL4 = 2000;
                public static final double kBothStop = 0;
        }

        public static final class AlgaeRPMSetpoints {
                public static final double kReefPickUpL123 = -.6;
                public static final double kProcessorDeliver = 3000;
                public static final double kBargeDeliver = 9000;
                public static final double kStop = 0;
        }

        public Command safePositionArmElevator(double degrees, double inches) {
                return Commands.sequence(
                                Commands.runOnce(() -> m_arm.setGoalDegrees(degrees)),
                                // Commands.waitUntil(() -> m_elevator.armClear),
                                Commands.waitUntil(() -> Units.radiansToDegrees(m_arm.armMotor.getEncoder()
                                                .getPosition()) < m_arm.armClearAngleDeg),
                                m_elevator.setGoalInchesCommand(inches));
        }

        public Command safePositionArmBarge(double degrees, double inches) {
                return Commands.sequence(
                                m_elevator.setGoalInchesCommand(inches),
                                //Commands.waitUntil(() -> m_elevator.atPosition()),
                                Commands.runOnce(() -> m_arm.setGoalDegrees(degrees)));
        }

        public Command safePositionArmElevatorL4(double degrees_first, double degrees_second, double inches) {
                return Commands.sequence(
                                Commands.runOnce(() -> m_arm.setGoalDegrees(degrees_first)),
                                // Commands.waitUntil(() -> m_elevator.armClear),
                                Commands.waitUntil(() -> Units.radiansToDegrees(m_arm.armMotor.getEncoder()
                                                .getPosition()) < m_arm.armClearAngleDeg),
                                m_elevator.setGoalInchesCommand(inches),
                                new WaitCommand(0.2),
                                Commands.waitUntil(() -> m_elevator.atPosition()),
                                Commands.runOnce(() -> m_arm.setGoalDegrees(degrees_second)));
        }

        public Command homeElevatorAndArm() {
                return Commands.sequence(
                                Commands.runOnce(() -> m_arm.setGoalDegrees(ArmSetpoints.kTravel)),
        
                                Commands.waitUntil(() -> Units.radiansToDegrees(m_arm.armMotor.getEncoder()
                                                .getPosition()) < m_arm.armClearAngleDeg),
                                m_elevator.setGoalInchesCommand(ElevatorSetpoints.kHome),
                                Commands.waitUntil(() -> m_elevator.atPosition()),
                                Commands.runOnce(() -> m_arm.setGoalDegrees(ArmSetpoints.kCoralStation)));
        }

        

        /**
         * Command to set the subsystem setpoint. This will set the arm and elevator to
         * their predefined
         * positions for the given setpoint.
         * 
         * 
         */
        public Command setSetpointCommand(Setpoint setpoint) {

                Command temp = Commands.none();

                switch (setpoint) {

                        case kTravel:
                                temp = safePositionArmElevator(ArmSetpoints.kTravel,
                                                ElevatorSetpoints.kTravel);
                                break;

                        case kCoralStation:
                                temp = safePositionArmElevator(ArmSetpoints.kCoralStation,
                                                ElevatorSetpoints.kCoralStation);
                                break;
                        case kLevel1:
                                temp = Commands.parallel(
                                                m_gamepieces.setTargetRPM(
                                                                CoralRPMSetpoints.kGamepieceReefPlaceL123),
                                                safePositionArmElevator(ArmSetpoints.kLevel1,
                                                                ElevatorSetpoints.kLevel1));
                                break;
                        case kLevel2:
                                temp = Commands.parallel(
                                                m_gamepieces.setTargetRPM(
                                                                CoralRPMSetpoints.kGamepieceReefPlaceL123),
                                                safePositionArmElevator(ArmSetpoints.kLevel2,
                                                                ElevatorSetpoints.kLevel2));
                                break;
                        case kLevel3:
                                temp = Commands.parallel(
                                                m_gamepieces.setTargetRPM(
                                                                CoralRPMSetpoints.kGamepieceReefPlaceL123),
                                                safePositionArmElevator(ArmSetpoints.kLevel3,
                                                                ElevatorSetpoints.kLevel3));
                                break;
                        case kLevel4:
                                temp = Commands.parallel(
                                                m_gamepieces.setTargetRPM(
                                                                CoralRPMSetpoints.kGamepieceReefPlaceL4),
                                                safePositionArmElevatorL4(ArmSetpoints.kLevel4_1,
                                                                ArmSetpoints.kLevel4_2,
                                                                ElevatorSetpoints.kLevel4));
                                break;
                        case kProcessorDeliver:
                                temp = safePositionArmElevator(ArmSetpoints.kProcessorDeliver,
                                                ElevatorSetpoints.kProcessorDeliver);
                                break;
                        case kAlgaeDeliverBarge:
                                temp = safePositionArmBarge(ArmSetpoints.kBargeDeliver,
                                                ElevatorSetpoints.kBarge);
                                break;
                        case kAlgaePickUpL2:
                                temp = safePositionArmElevator(ArmSetpoints.kAlgaeIntake,
                                                ElevatorSetpoints.kLevelAlgaeL2);
                                break;
                        case kAlgaePickUpL3:
                                temp = safePositionArmElevator(ArmSetpoints.kAlgaeIntake,
                                                ElevatorSetpoints.kLevelAlgaeL3);
                                break;
                }
                return temp;

        }

        public Command deliverCoralL4() {
                return Commands.sequence(
                                setSetpointCommand(Setpoint.kLevel4),
                                m_gamepieces.deliverCoralCommand(),
                                homeElevatorAndArm());

        }

        // .andThen(Commands.runOnce(() ->
        // m_ls.setViewThreeSolidColor(setpoint.ordinal())));
}
