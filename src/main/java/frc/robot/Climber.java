package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber {

    public static WPI_TalonFX m_climber;
    public static boolean inited = false;
    public static boolean inited_outer = false;

    public Climber() {
        if (!inited) {
            m_climber = new WPI_TalonFX(15);
            m_climber.configFactoryDefault();
            m_climber.configNeutralDeadband(0.04);
            m_climber.setNeutralMode(NeutralMode.Brake);
            m_climber.configVoltageCompSaturation(12);
            m_climber.enableVoltageCompensation(true);
            m_climber.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.01));
            m_climber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            m_climber.setSelectedSensorPosition(0);
            inited = true;
        }
    }

    public void set(double speed) {
        m_climber.set(speed);
    }

    public double get() {
        return m_climber.get();
    }

    public double getPosition() {
        return m_climber.getSelectedSensorPosition();
    }

    public double getCurrent() {
        return m_climber.getStatorCurrent();
    }

    public double getVelocity() {
        return m_climber.getSelectedSensorVelocity();
    }

    public void reset() {
        m_climber.setSelectedSensorPosition(0);
        m_climber.set(0);
    }

    public void setSoftLimit(boolean enable) {
        if(enable) {
            m_climber.configForwardSoftLimitThreshold(0);
            m_climber.configReverseSoftLimitThreshold(-421000);
            m_climber.configForwardSoftLimitEnable(true);
            m_climber.configReverseSoftLimitEnable(true);
        }
        else {
            m_climber.configForwardSoftLimitEnable(false);
            m_climber.configReverseSoftLimitEnable(false);
        }
    }

}
