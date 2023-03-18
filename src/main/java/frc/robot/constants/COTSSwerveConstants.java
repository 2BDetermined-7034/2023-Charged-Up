package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSSwerveConstants {
    public final double maxSpeed;
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final double driveKP;
    public final double driveKI;
    public final double driveKD;
    public final double driveKF;
    public final double driveKS;
    public final double driveKV;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final boolean canCoderInvert;
    public double driveKA;

    public COTSSwerveConstants(
            double maxSpeed,
            double wheelDiameter,
            double angleGearRatio,
            double driveGearRatio,
            double angleKP,
            double angleKI,
            double angleKD,
            double angleKF,
            double driveKP,
            double driveKI,
            double driveKD,
            double driveKF,
            double driveKS,
            double driveKV,
            double driveKA,
            boolean driveMotorInvert,
            boolean angleMotorInvert,
            boolean canCoderInvert) {
        this.maxSpeed = maxSpeed;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveKP = driveKP;
        this.driveKI = driveKI;
        this.driveKD = driveKD;
        this.driveKF = driveKF;
        this.driveKS = driveKS;
        this.driveKV = driveKV;
        this.driveKA = driveKA;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderInvert = canCoderInvert;
    }

    /**
     * DriveSubsystem Drive Specialties - MK4i Module
     */
    public static COTSSwerveConstants SDSMK4i(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        double angleGearRatio = ((150.0 / 7.0));

        //TODO: Tune all of these gains
        double angleKP = 0.01;
        double angleKI = 0.0000;
        double angleKD = 0.0;
        double angleKF = 0.0;

        double driveKP = 0.0000;
        double driveKI = 0.0;
        double driveKD = 0.0;
        double driveKF = 0.0;

        double driveKS = 0.18684;
        double driveKV = 2.6395;
        double driveKA = 0.38108;

        boolean driveMotorInvert = true;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;

        return new COTSSwerveConstants(
                (5880 / 60.0 / driveGearRatio * wheelDiameter * Math.PI),
                wheelDiameter,
                angleGearRatio,
                driveGearRatio,
                angleKP,
                angleKI,
                angleKD,
                angleKF,
                driveKP,
                driveKI,
                driveKD,
                driveKF,
                driveKS,
                driveKV,
                driveKA,
                driveMotorInvert,
                angleMotorInvert,
                canCoderInvert);
    }

    public static class driveGearRatios {
        /* SDS MK4i */
        /**
         * SDS MK4i - 8.14 : 1
         */
        public static final double SDSMK4i_L1 = (8.14);
        /**
         * SDS MK4i - 6.75 : 1
         */
        public static final double SDSMK4i_L2 = (6.75);
        /**
         * SDS MK4i - 6.12 : 1
         */
        public static final double SDSMK4i_L3 = (6.12);
    }
}