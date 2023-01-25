// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;

import java.util.Map;

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

    public static class Vision {
        public static final Transform2d camToRobot = new Transform2d();
    }
}
