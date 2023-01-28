package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;


public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
    public static class Drivebase {

        public static class Measurements {
            public static final double width = Units.inchesToMeters(32);
            public static final double length = Units.inchesToMeters(32);
            public static final double driveRatio = COTSSwerveConstants.driveGearRatios.SDSMK4i_L2;

        }
        public static class MotorIDs {
            public static final int flDrive = 3;
            public static final int flSteer = 13;
            public static final int flEncoder = 1;
            public static final Rotation2d flOffset = Rotation2d.fromDegrees(173.1);
            //public static final Rotation2d flOffset = Rotation2d.fromDegrees(263.1 + 0); //266.6

            public static final int frDrive = 12;
            public static final int frSteer = 11;
            public static final int frEncoder = 2;
            public static final Rotation2d frOffset = Rotation2d.fromDegrees(252.8 + 0);
            //public static final Rotation2d frOffset = Rotation2d.fromDegrees(68.7 - 2);

            public static final int blDrive = 5;
            public static final int blSteer = 14;
            public static final int blEncoder = 4;
            public static final Rotation2d blOffset = Rotation2d.fromDegrees(217.3 - 0);

//            public static final Rotation2d blOffset = Rotation2d.fromDegrees(160.83 - 1);

            public static final int brDrive = 2;
            public static final int brSteer = 6;
            public static final int brEncoder = 3;
            public static final Rotation2d brOffset = Rotation2d.fromDegrees(155.4 + 0);
//            public static final Rotation2d brOffset = Rotation2d.fromDegrees(266.5 + 0);
        }
        public static class MotorConfig {
            public static int angleSmartCurrentLimit = 25;
            public static int angleSecondaryCurrentLimit = 60;
            public static CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
            public static int driveSmartCurrentLimit = 25;
            public static int driveSecondaryCurrentLimit = 60;
            public static CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

            public static final double openLoopRamp = 0.25;
            public static final double closedLoopRamp = 0.0;

        }

        public static class Auto {
            public static final double maxVelocity = 1;
            public static final double maxAcceleration = 2;
            public static final double kP = 0.0;
        }

    }

    public static class VisionConstants {
        public static final Transform2d camToRobot = new Transform2d();
    }

    public static class ArmConstants {

        public static final int motor1ID = 15;
        public static final int motor2ID = 9;
        //Length of Segments
        public static final double l1 = Units.inchesToMeters(38.5);
        public static final double l2 = Units.inchesToMeters(33.5);
        public static final double fullRadius = l1 + l2;

        //Mass of segments
        public static final double m1 = 1.3;
        public static final double m2 = 0.53;

        //Distance from pivot to CG for each segment
        public static final double r1 = 21.64 * .0254;
        public static final double r2 = 26.70 * .0254;

        //Moment of inertia about CG for each segment
        public static final double I1 = .118;
        public static final double I2 = .031;

        //Gearing of each segment
        public static final double G1 = 80.;
        public static final double G2 = 112.5;

        public static final double S1 = 2 * Math.PI * (1/G1);
        public static final double S2 = 2 * Math.PI * (1/G2);

        //Number of motors in each gearbox
        public static final int N1 = 1;
        public static final int N2 = 1;

        //Gravity
        public static final double g = 9.81;

        public static final double stall_torque = 3.28;
        public static final double free_speed = 5820 * 2.0 * Math.PI / 60.0;
        public static final double stall_current = 181;

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
    }
}