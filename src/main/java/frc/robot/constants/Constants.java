// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm.ArmConfig;
import frc.robot.subsystems.Arm.ArmDynamics;
import frc.robot.subsystems.Arm.ArmState;

import java.util.HashMap;

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
        /**
         * TODO redo all constants here, probably from fusion 360, make angles line up with new classes, set proper minMax angles, etc
         */
        public static class ArmSetPoints {
            public static ArmState passThrough = new ArmState(Units.degreesToRadians(125), Units.degreesToRadians(190));
            public static ArmState preIntake = new ArmState(Units.degreesToRadians(112), Units.degreesToRadians(244), 0, 0.2);

            public static ArmState intake = new ArmState(Units.degreesToRadians(122), Units.degreesToRadians(254));
            public static ArmState tuck = new ArmState(Units.degreesToRadians(106), Units.degreesToRadians(223));
            public static ArmState midBack = new ArmState(Units.degreesToRadians(132), Units.degreesToRadians(5));
            //public static ArmState midBack = new ArmState(Units.degreesToRadians(95), Units.degreesToRadians(30));
            public static ArmState frontMid = new ArmState(Units.degreesToRadians(138), Units.degreesToRadians(170));
            public static ArmState high = new ArmState(Units.degreesToRadians(95), Units.degreesToRadians(30));
            public static ArmState startCone = new ArmState(Units.degreesToRadians(95), Units.degreesToRadians(240));

        }

        public static final double kEncoder1Offset = 0.8018;
        public static final double kEncoder2Offset = 0.4797;

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
        public static final double kG1 = 0.47d, kV1 = 3.47d, kA1 = 0.03d;
        public static final double kG2 = 0.28, kV2 = 2.60, kA2 = .03;


        /**
         * These are Mechanical Advantage Arm Config Classes
         * TODO move these to JSON files
         * Also I have no idea what "interior points" is for the Solver Config :/
         */
        public static final HashMap<String, ArmConfig.Constraint> constraintHashMap = new HashMap<>();
        public static final ArmConfig armConfig = new ArmConfig(
                new Translation2d(),
                new ArmConfig.JointConfig(
                        m1,
                        l1,
                        I1,
                        r1,
                        0,
                        Units.degreesToRadians(140),
                        new ArmConfig.MotorConfig(
                                DCMotor.getNeo550(N1),
                                G1
                        )
                ),
                new ArmConfig.JointConfig(
                        m2,
                        l2,
                        I2,
                        r2,
                        0,
                        Units.degreesToRadians(360),
                        new ArmConfig.MotorConfig(
                                DCMotor.getNeo550(N2),
                                G2
                        )
                ),
                new ArmConfig.JointConfig(0,0,0,0,0,0,new ArmConfig.MotorConfig(DCMotor.getNeo550(1), 0)),

                new ArmConfig.SolverConfig(
                        15, 12, 12, 30
                ),
                constraintHashMap
        );

        public static final ArmDynamics dynamics = new ArmDynamics(armConfig);
    }
}