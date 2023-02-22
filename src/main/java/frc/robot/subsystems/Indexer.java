package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Indexer extends SubsystemBase implements SubsystemLogging {
    public CANSparkMax indexerMotorLeft;
    public CANSparkMax indexerMotorRight;
    private MotorControllerGroup indexerMotors;

    public Indexer() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        indexerMotorLeft = new CANSparkMax(Constants.Intake.indexerMotorLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexerMotorRight = new CANSparkMax(Constants.Intake.indexerMotorRight, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexerMotorRight.setInverted(false);
        indexerMotorLeft.setInverted(true);

        indexerMotors = new MotorControllerGroup(indexerMotorLeft, indexerMotorRight);

    }

    public void runIndexerCounterclockwise() {
        indexerMotors.setVoltage(Constants.Intake.indexerSpeed);
    }

    public void runIndexerClockwise() {
        indexerMotors.setVoltage(-Constants.Intake.indexerSpeed);
    }
    public void stopIndexer() {
        indexerMotors.setVoltage(0);
    }

    @Override
    public void updateLogging() {
        log("Indexer Speed", indexerMotorLeft.getAppliedOutput());
    }

    @Override
    public void periodic() {
        updateLogging();
    }
}

