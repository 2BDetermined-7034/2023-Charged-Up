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


            public static final int frDrive = 12;
            public static final int frSteer = 11;
            public static final int frEncoder = 2;


            public static final int blDrive = 5;
            public static final int blSteer = 14;
            public static final int blEncoder = 4;


            public static final int brDrive = 2;
            public static final int brSteer = 6;
            public static final int brEncoder = 3;

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

        public static final int motor1ID = 12;
        public static final int motor2ID = 13;
        //Length of Segments
        public static final double l1 = 46.25 * .0254;
        public static final double l2 = 41.80 * .0254;

        //Mass of segments
        public static final double m1 = 9.34 * .4536;
        public static final double m2 = 9.77 * .4536;

        //Distance from pivot to CG for each segment
        public static final double r1 = 21.64 * .0254;
        public static final double r2 = 26.70 * .0254;

        //Moment of inertia about CG for each segment
        public static final double I1 = 2957.05 * .0254 * .0254 * .4536;
        public static final double I2 = 2824.70 * .0254 * .0254 * .4536;

        //Gearing of each segment
        public static final double G1 = 140.;
        public static final double G2 = 90.;

        //Number of motors in each gearbox
        public static final int N1 = 1;
        public static final int N2 = 2;

        //Gravity
        public static final double g = 9.81;

        public static final double stall_torque = 3.36;
        public static final double free_speed = 5880.0 * 2.0 * Math.PI / 60.0;
        public static final double stall_current = 166;

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