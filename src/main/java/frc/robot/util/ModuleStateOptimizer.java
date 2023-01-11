package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Even though
public class ModuleStateOptimizer {

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle =
                placeInAppropriate0To360Scope(
                        currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        // If we rotate more than 90 degrees, we need to reverse the speed
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {
                targetAngle -= 180;
            } else {
                targetAngle += 180;
            }
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) { // If we are in the positive
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else { // If we are in the negative
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) { // Isn't there a better way?
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        // If we have to turn more than 180 degrees rotate the other way to avoid unnecessary
        // rotation.
        if (newAngle - scopeReference
                > 180) { // If were we are vs were we are going to turn to is over 180 degrees
            newAngle -= 360; // Decrease it by 360 to rotate the other way around
        } else if (newAngle - scopeReference
                < -180) { // Same as above for when we are in the negative
            newAngle += 360;
        }
        return newAngle;
    }
}