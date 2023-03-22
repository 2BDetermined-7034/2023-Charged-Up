package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.ArmState;


public class SetArmCommandWithConstraints extends SetArmCommand {

    protected TrapezoidProfile.Constraints constraint1;
    protected TrapezoidProfile.Constraints constraint2;

    /**
     * Same as setArmCommand but with a Velocity/Acceleration Constraint.
     *
     * @param arm
     * @param drive
     * @param goalState
     * @param toggleSpeedOnEnd
     */
    public SetArmCommandWithConstraints(Arm arm, SwerveDrive drive, ArmState goalState, boolean toggleSpeedOnEnd, TrapezoidProfile.Constraints constraint1, TrapezoidProfile.Constraints constraint2) {
        super(arm, drive, goalState, toggleSpeedOnEnd);
        this.constraint1 = constraint1;
        this.constraint2 = constraint2;
    }


    @Override
    public void initialize() {
        drive.setSpeedMulti(0.2);
        arm.setGoalState(goalState);
        arm.setConstraints(
                constraint1,constraint2
        );
    }

    @Override
    public void end(boolean interrupted) {
        if(toggleSpeedOnEnd) {
            drive.setSpeedMulti(1);
        }
        arm.setConstraints(Constants.ArmConstants.shoulderConstraints, Constants.ArmConstants.elbowConstraints);
    }
}
