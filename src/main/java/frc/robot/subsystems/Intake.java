package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {

    public CANSparkMax motor1;
    public CANSparkMax motor2;

    private final RelativeEncoder m_Encoder1, m_Encoder2;
    private final ProfiledPIDController controller1 = new ProfiledPIDController(0.4,0,0, new TrapezoidProfile.Constraints(1,2));

    public MotorControllerGroup motorControllerGroup;


    public Intake() {
        motor1 = new CANSparkMax(Constants.Intake.intakeMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.Intake.intakeMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_Encoder1 = motor1.getEncoder();
        m_Encoder2 = motor2.getEncoder();
        double s1 = 2 * Math.PI * (1d/1); //Replace denominator iwth gear ratio
        double s2 = 2 * Math.PI * (1d/1); //Replace denominator iwth gear ratio
        m_Encoder1.setPositionConversionFactor(s1/*TODO replace with Gear Ratio*/);
        m_Encoder2.setPositionConversionFactor(s2);
        m_Encoder1.setPosition(0);
        m_Encoder2.setPosition(0);
        m_Encoder1.setVelocityConversionFactor(s1/60);
        m_Encoder2.setVelocityConversionFactor(s2/60);
        controller1.enableContinuousInput(0,2 * Math.PI);

        motorControllerGroup = new MotorControllerGroup(motor1, motor2);

    }

    public void runIntakeForward() {
        motorControllerGroup.setVoltage(Constants.Intake.intakeSpeed);
    }

    public void runIntakeBackwards() {
        motorControllerGroup.setVoltage(-Constants.Intake.intakeSpeed);
    }

    public void setCoterminal() {
        motor1.setVoltage(-MathUtil.clamp(controller1.calculate(m_Encoder1.getPosition() % 180, 0), -12, 12));

        motor2.setVoltage(-MathUtil.clamp(controller1.calculate(m_Encoder2.getPosition() % 180, 0), -12, 12));
    }


}

