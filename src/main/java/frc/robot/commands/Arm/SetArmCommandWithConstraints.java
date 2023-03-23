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
     * @param arm arm
     * @param goalState arm goalstate
     * @param constraint1 shoulder mation profile constraint
     * @param constraint2 elbow motion profile constraint
     *
     */
    public SetArmCommandWithConstraints(Arm arm, ArmState goalState, TrapezoidProfile.Constraints constraint1, TrapezoidProfile.Constraints constraint2) {
        super(arm, goalState);
        this.constraint1 = constraint1;
        this.constraint2 = constraint2;
    }


    @Override
    public void initialize() {
        arm.setGoalState(goalState);
        arm.setConstraints(
                constraint1,constraint2
        );
    }

    @Override
    public void end(boolean interrupted) {
        arm.setConstraints(Constants.ArmConstants.shoulderConstraints, Constants.ArmConstants.elbowConstraints);
    }
}
