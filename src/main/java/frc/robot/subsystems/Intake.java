package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {

    public CANSparkMax motor1;
    public CANSparkMax motor2;

    private final RelativeEncoder m_Encoder1, m_Encoder2;
    private final PIDController controller1 = new PIDController(0.4,0,0);

    public MotorControllerGroup motorControllerGroup;

    private final DoubleSolenoid solenoid;


    public Intake() {
        motor1 = new CANSparkMax(Constants.Intake.intakeMotorLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.Intake.intakeMotorRight, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_Encoder1 = motor1.getEncoder();
        m_Encoder2 = motor2.getEncoder();
        double s1 = 2 * Math.PI * (1d/1); //Replace denominator iwth gear ratio
        double s2 = 2 * Math.PI * (1d/1); //Replace denominator iwth gear ratio
        m_Encoder1.setPositionConversionFactor(s1/*TODO replace with Gear Ratio*/);
        m_Encoder2.setPositionConversionFactor(s2);
        m_Encoder1.setPosition(0);
        m_Encoder2.setPosition(0);
        controller1.enableContinuousInput(0,2 * Math.PI);

        motorControllerGroup = new MotorControllerGroup(motor1, motor2);

        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.intakeFC, Constants.Intake.intakeRC);

    }

    public void runIntakeForward() {
        motorControllerGroup.setVoltage(Constants.Intake.intakeSpeed);
    }

    public void runIntakeBackwards() {
        motorControllerGroup.setVoltage(-Constants.Intake.intakeSpeed);
    }

    public void runIntake(double forward, double reverse) {
        motorControllerGroup.setVoltage(forward > reverse && (forward > .05 || reverse  > .05) ? forward  * 6 : reverse  * 3);
    }

    /**
     *
     * @param position true: forward, false: reverse
     */
    public void setSolenoid(boolean position) {
        if(position) {
            solenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void toggleSolenoid() {
        if (solenoid.get().equals(DoubleSolenoid.Value.kForward)) {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            solenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void setCoterminal() {
        motor1.setVoltage(-MathUtil.clamp(controller1.calculate(m_Encoder1.getPosition() % 180, 0), -12, 12));

        motor2.setVoltage(-MathUtil.clamp(controller1.calculate(m_Encoder2.getPosition() % 180, 0), -12, 12));
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }


}

