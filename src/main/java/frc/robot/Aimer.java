package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Aimer {

    public static CANSparkMax m_aim = new CANSparkMax(11, MotorType.kBrushless);
    public static boolean inited = false;
    public static boolean inited_outer = false;

    public static SparkMaxPIDController pid;
    public static RelativeEncoder encoder;

    public Aimer() {
        if (!inited) {
            m_aim.restoreFactoryDefaults();
            m_aim.setInverted(false);

            pid = m_aim.getPIDController();
            pid.setP(1);
            pid.setI(1e-4);
            pid.setD(1);
            pid.setIZone(0.5);
            pid.setFF(0);
            pid.setOutputRange(-1, 1);

            encoder = m_aim.getEncoder();
            encoder.setPositionConversionFactor(0.3490658503988659);
            encoder.setVelocityConversionFactor(0.005817764173314432);
            encoder.setPosition(0);

            m_aim.setSoftLimit(SoftLimitDirection.kForward, 5.45f);
            m_aim.setSoftLimit(SoftLimitDirection.kReverse, 0.15f);

            m_aim.enableSoftLimit(SoftLimitDirection.kForward, true);
            m_aim.enableSoftLimit(SoftLimitDirection.kReverse, true);

            inited = true;
        }
    }

    public void set(double speed) {
        m_aim.set(speed);
    }

    public double get() {
        return m_aim.get();
    }

    public double getCurrent() {
        return m_aim.getOutputCurrent();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void setPosition(double pos) {
        pos = Math.min(Math.max(pos, 0.15), 5.45);
        pid.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public void setSoftLimit(boolean enable) {
        m_aim.enableSoftLimit(SoftLimitDirection.kForward, enable);
        m_aim.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    public void reset() {
        encoder.setPosition(0);
        m_aim.set(0);
    }
}
