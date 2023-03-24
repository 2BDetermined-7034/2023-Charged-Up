// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmState;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kOperatorControllerPort = 1;
    }

    public static class GravityClaw {
        //blue
        public static final int grabberFC = 7;
        public static final int grabberRC = 6;
    }

    public static class Drivebase {
        public static class Measurements {
            public static final double width = Units.inchesToMeters(32);
            public static final double length = Units.inchesToMeters(32);
            public static final double driveRatio = COTSSwerveConstants.driveGearRatios.SDSMK4i_L2;
        }

        public static class MotorIDs {

            // practice robot settings:
            public static final int flDrive = 12;
            public static final int flSteer = 13;
            public static final int flEncoder = 2;
            public static final Rotation2d flOffset = Rotation2d.fromDegrees(-21.533201217651367);


            // Practice robot settings (done):
            public static final int frDrive = 4;

            public static final int frSteer = 3;
            public static final int frEncoder = 1;
            public static final Rotation2d frOffset = Rotation2d.fromDegrees(-65.83008575439453);

            // done
            public static final int blDrive = 10;
            public static final int blSteer = 9;
            public static final int blEncoder = 3;
            public static final Rotation2d blOffset = Rotation2d.fromDegrees(-1.23046875);

            // done
            public static final int brDrive = 6;
            public static final int brSteer = 5;
            public static final int brEncoder = 4;
            public static final Rotation2d brOffset = Rotation2d.fromDegrees(71.45);
        }

        public static class MotorConfig {
            public static final double openLoopRamp = 0.25;
            public static final double closedLoopRamp = 0.0;
            public static int angleSmartCurrentLimit = 25;
            public static int angleSecondaryCurrentLimit = 60;
            public static CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
            public static int driveSmartCurrentLimit = 25;
            public static int driveSecondaryCurrentLimit = 60;
            public static CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

        }

        public static class Auto {
            public static final double maxVelocity = 1;
            public static final double maxAcceleration = 2;
            public static final double kP = 0.0;
        }

    }


    public static class Intake {
        public static final int intakeMotorLeft = 11;
        public static final int intakeMotorRight = 2;
        public static final int indexerMotorLeft = 14;
        public static final int indexerMotorRight = 1;

        public static final double indexerSpeed = 0.4;
        public static final double intakeSpeed = 6.0;
        //red
        public static final int intakeFC = 8;
        public static final int intakeRC = 9;
    }

    public static class Vision {
        public static final double goalHeighInches = 30;
        public static final double limeligtLensHeighInches = 20;
        public static final double limeLightMountAngleDegrees = 0;
    }

    public static class ArmConstants {
        public static final TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(5, 4);
        public static final TrapezoidProfile.Constraints elbowConstraints = new TrapezoidProfile.Constraints(5, 2.5);
        public static class ArmSetPoints {
            public static ArmState limit = new ArmState(Units.degreesToRadians(70), Units.degreesToRadians(90), 0, 0.2);

            public static ArmState passThrough = new ArmState(Units.degreesToRadians(105), Units.degreesToRadians(230), 0, 0);
            public static ArmState passThroughOut = new ArmState(Units.degreesToRadians(106), Units.degreesToRadians(223), 0, 0);

            public static ArmState preIntake = new ArmState(Units.degreesToRadians(86), Units.degreesToRadians(245), 0, 0.2);

            public static ArmState intake = new ArmState(Units.degreesToRadians(100), Units.degreesToRadians(255));
            public static ArmState tuck = new ArmState(Units.degreesToRadians(93), Units.degreesToRadians(193));
            public static ArmState midBack = new ArmState(Units.degreesToRadians(109), Units.degreesToRadians(5));
            //public static ArmState midBack = new ArmState(Units.degreesToRadians(95), Units.degreesToRadians(30));
            public static ArmState shelf = new ArmState(Units.degreesToRadians(117), Units.degreesToRadians(9));
            public static ArmState high = new ArmState(Units.degreesToRadians(79), Units.degreesToRadians(28));
            public static ArmState startCone = new ArmState(Units.degreesToRadians(95), Units.degreesToRadians(240));

        }

        public static final double kEncoder1Offset = (175.2735 + 90)/360;
        public static final double kEncoder2Offset = 313.8125/360;

        public static final double kMaxArmOverrideSpeedShoulder = 2;
        public static final double kMaxArmOverrideSpeedDistal = 2.5;
        public static final int motor1ID = 8;
        public static final int motor2ID = 7;

        //Length of Segments
        public static final double l1 = Units.inchesToMeters(38.5);
        public static final double l2 = Units.inchesToMeters(33.5);

        //Mass of segments
        public static final double m1 = 1.3;
        public static final double m2 = 0.53 + 0.9;

        //Distance from pivot to CG for each segment
        public static final double r1 = l1 / 2;
        public static final double r2 = l2 / 2;

        //Moment of inertia about CG for each segment
        public static final double I1 = .118;
        public static final double I2 = .031;

        //Gearing of each segment
        public static final double G1 = 5 * 4 * 2 * (80d/18d);
        public static final double G2 = 5 * 3 * 2 * (80d/18d);

        public static final double S1 = 2 * Math.PI * (1 / G1);
        public static final double S2 = 2 * Math.PI * (1 / G2);

        //Number of motors in each gearbox
        public static final int N1 = 1;
        public static final int N2 = 1;

        //ArmFeedForward Gains
        public static final double kG1 = 0.47d, kV1 = 3.42d, kA1 = 0.03d;
        public static final double kG2 = 0.28, kV2 = 2.59, kA2 = .03;

        //Gravity
        public static final double g = 9.81;

        public static final double stall_torque = 2.6;
        public static final double free_speed = 5676 * 2.0 * Math.PI / 60.0;
        public static final double stall_current = 105;

        public static final double Rm = 12.0 / stall_current;

        public static final double Kv = free_speed / 12.0;
        public static final double Kt = stall_torque / stall_current;

        //K3*Voltage - K4*velocity = motor torque
        public static final Matrix<N2, edu.wpi.first.math.numbers.N2> K3 = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                N1 * G1,
                0,
                0,
                N2 * G2
        ).times(Kt / Rm);

        public static final Matrix<N2, N2> K4 = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                G1 * G1 * N1,
                0,
                0,
                G2 * G2 * N2
        ).times(Kt / Kv / Rm);

        final static double B1 = G1 * N1 * Kt / Rm;
        final static double B4 = G2 * N2 * Kt / Rm;
        public static final Matrix<N2, N2> B = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                B1,
                0,
                0,
                B4
        );
    }
}