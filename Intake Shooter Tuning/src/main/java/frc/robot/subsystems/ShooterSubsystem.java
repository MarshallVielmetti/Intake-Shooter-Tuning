package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor1 = new CANSparkMax(kShooterMotorID1, MotorType.kBrushless); // Bottom Motor
    private final CANSparkMax m_motor2 = new CANSparkMax(kShooterMotorID2, MotorType.kBrushless); // Top Motor

    private final CANEncoder m_encoder = m_motor1.getEncoder();

    private final CANPIDController m_pidController = m_motor1.getPIDController();

    private double m_velocity = 0;
    private boolean m_enabled = false;

    public ShooterSubsystem() {
        m_motor1.setInverted(kMotor1Inverted);
        m_motor2.setInverted(kMotor2Inverted);

        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor2.follow(m_motor1); // Get rid of if we want to try out different topspin values. This just makes it
                                   // easier to be more consistent.

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        /**
         * Smart Motion coefficients are set on a CANPIDController object
         *
         * <p>
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid
         * controller in
         * Smart Motion mode - setSmartMotionMinOutputVelocity() will put a lower bound
         * in RPM of the
         * pid controller in Smart Motion mode - setSmartMotionMaxAccel() will limit the
         * acceleration in
         * RPM^2 of the pid controller in Smart Motion mode -
         * setSmartMotionAllowedClosedLoopError()
         * will set the max allowed error for the pid controller in Smart Motion mode
         */
        m_pidController.setSmartMotionMaxVelocity(kMaxVel, kSmartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(kMinVel, kSmartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(kMaxAcc, kSmartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, kSmartMotionSlot);
    }

    @Override
    public void periodic() {

        if (kDebug) {
            this.doDebug();
        }
    }

    public void setDesiredVelocity(double velocity) {
        if (m_enabled) {
            m_pidController.setReference(velocity, ControlType.kVelocity);
        } else {
            m_pidController.setReference(0, ControlType.kVelocity);
        }
    }

    public void toggleEnabled() {
        if (m_enabled) {
            setDesiredVelocity(0); // note that passed value is irrelevant
        } else {
            setDesiredVelocity(m_velocity);
        }
    }

    private void doDebug() {
        double setVel = SmartDashboard.getNumber("Shooter Set Velocity", 0);
        if (setVel != m_velocity) {
            this.m_velocity = setVel;
            this.setDesiredVelocity(m_velocity);
        }

        SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
    }

    private void isAtSpeed() {
        if (m_encoder.getVelocity() > .95 * m_velocity || m_encoder.getVelocity() < 1.05 * m_velocity) {
            SmartDashboard.putBoolean("Shooter At Speed", true);
        } else {
            SmartDashboard.putBoolean("Shooter At Speed", false);
        }
    }

}
