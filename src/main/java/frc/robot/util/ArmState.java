package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.Constants.ArmConstants.l1;
import static frc.robot.constants.Constants.ArmConstants.l2;


public class ArmState {

    private Rotation2d theta1;
    private Rotation2d theta2;
    private double omega1;
    private double omega2;
    private double accel1;
    private double accel2;

    public ArmState(Rotation2d theta1, Rotation2d theta2, double omega1, double omega2, double accel1, double accel2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.omega1 = omega1;
        this.omega2 = omega2;
        this.accel1 = accel1;
        this.accel2 = accel2;
    }

    public ArmState(Rotation2d theta1, Rotation2d theta2, double omega1, double omega2) {
        this(theta1, theta2, omega1, omega2, 0, 0);
    }

    public ArmState(double theta1, double theta2) {
        this(Rotation2d.fromRadians(theta1), Rotation2d.fromRadians(theta2), 0, 0);
    }

    /**
     * Checks if an inverse-kinematics solution exists for the given point in space
     *
     * @param x position in space of end effector
     * @param y position in space of end effector
     * @return whether the solution exists
     */
    public boolean solutionExists(double x, double y) {
        double sqrt = Math.sqrt(x * x + y * y);
        return !(l1 + l2 < sqrt || Math.abs(l1 - l2) > sqrt);
    }

    /**
     * Getter Methods
     */
    public double getTheta1() {
        return this.theta1.getRadians();
    }

    public double getTheta2() {
        return this.theta2.getRadians();
    }

    public double getOmega1() {
        return this.omega1;
    }

    public double getOmega2() {
        return this.omega2;
    }

    public double getAlpha1() {
        return this.accel1;
    }

    public double getAlpha2() {
        return this.accel2;
    }

    public ArmState clear() {
        this.omega1 = 0;
        this.omega2 = 0;
        this.accel1 = 0;
        this.accel2 = 0;
        return this;
    }

    public ArmState incrementDegrees(double theta1, double theta2) {
        this.theta1 = Rotation2d.fromDegrees(this.theta1.getDegrees() + theta1);
        this.theta2 = Rotation2d.fromDegrees(this.theta2.getDegrees() + theta2);
        return this;

    }

}