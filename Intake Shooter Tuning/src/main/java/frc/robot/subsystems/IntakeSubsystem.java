
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor1 = new CANSparkMax(kIntakeMotorID1, MotorType.kBrushless);
    private final CANSparkMax m_motor2 = new CANSparkMax(kIntakeMotorID2, MotorType.kBrushless);

    public IntakeSubsystem() {
        m_motor1.setInverted(kMotor1Inverted);
        m_motor2.setInverted(kMotor2Inverted);

        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor2.follow(m_motor1);
    }

    public void intake() {
        m_motor1.setVoltage(kIntakeVolts);
    }

    public void stop() {
        m_motor1.set(0);
    }
}