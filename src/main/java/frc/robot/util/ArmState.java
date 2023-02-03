package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.Constants.ArmConstants.l1;
import static frc.robot.constants.Constants.ArmConstants.l2;


public class ArmState {

    private final Rotation2d theta1, theta2;
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
    public ArmState(Rotation2d theta1, Rotation2d theta2) {
        this(theta1, theta2, 0, 0);
    }
    public ArmState(double theta1, double theta2) {this(Rotation2d.fromRadians(theta1), Rotation2d.fromRadians(theta2), 0, 0);}
    public ArmState(double x, double y, boolean invert, ArmState currentState) {

        Rotation2d theta1;
        Rotation2d theta2;

        if (solutionExists(x, y)) {
            theta2 = Rotation2d.fromRadians(Math.acos((x * x + y * y - (l1 * l1 + l2 * l2)) / (2 * l1 * l2)));
            theta1 = Rotation2d.fromRadians(Math.atan2(y, x) + (invert ? -1 : 1) * Math.atan2(l2 * Math.sin(theta2.getRadians()), l1 + l2 * Math.cos(theta2.getRadians())));
            this.theta1 = theta1;
            this.theta2 = theta2;
            return;
        }

        this.theta1 = currentState.theta1;
        this.theta2 = currentState.theta2;
    }

    /**
     * Checks if an inverse-kinematics solution exists for the given point in space
     * @param x position in space of end effector
     * @param y position in space of end effector
     * @return whether the solution exists
     */
    private boolean solutionExists(double x, double y) {
        double sqrt = Math.sqrt(x * x + y * y);
        return !(l1 + l2 < sqrt || Math.abs(l1 - l2) > sqrt);
    }

    /** Getter Methods */
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

}