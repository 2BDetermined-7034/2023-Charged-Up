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
        indexerMotorLeft.setSmartCurrentLimit(15);

        indexerMotorRight.setSmartCurrentLimit(15);

        indexerMotorRight.setInverted(true);
        indexerMotorLeft.setInverted(false);

        indexerMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        indexerMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        indexerMotors = new MotorControllerGroup(indexerMotorLeft, indexerMotorRight);

    }

    public void runIndexer(double speed) {
        indexerMotors.set(speed);
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

