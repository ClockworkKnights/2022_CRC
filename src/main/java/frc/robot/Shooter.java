package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter {

    public static WPI_TalonFX m_left = new WPI_TalonFX(13);
    public static WPI_TalonFX m_right = new WPI_TalonFX(14);

    // private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    public static boolean inited = false;

    private double kp;
    private double ki;
    private double kd;

    private double kf;
    private PIDController pid;

    public Shooter() {
        if (!inited) {
            m_left.configFactoryDefault();
            m_left.setNeutralMode(NeutralMode.Coast);
            m_left.setInverted(false);
            m_left.configVoltageCompSaturation(12);
            m_left.enableVoltageCompensation(true);
            m_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            m_left.selectProfileSlot(0, 0);
            m_left.set(0);

            m_right.configFactoryDefault();
            m_right.setNeutralMode(NeutralMode.Coast);
            m_right.setInverted(true);
            m_right.configVoltageCompSaturation(12);
            m_right.enableVoltageCompensation(true);
            m_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            m_right.selectProfileSlot(0, 0);
            m_right.set(0);

            m_left.config_kP(0, 0); // 0.1
            m_left.config_kI(0, 0.00005);// 0.0006);
            m_left.config_kD(0, 0); // 0.02
            m_left.config_kF(0, 0.052);
            m_left.config_IntegralZone(0, 3000);

            m_right.config_kP(0, 0);
            m_right.config_kI(0, 0.00005);// 0.0006);
            m_right.config_kD(0, 0);
            m_right.config_kF(0, 0.052);
            m_right.config_IntegralZone(0, 3000);

            m_right.follow(m_left);

            kp = 0;
            ki = 0;
            kd = 0;
            kf = 0;

            pid = new PIDController(kp, ki, kd);

            inited = true;
        }
    }

    public double getTempLeft() {
        return m_left.getTemperature();
    }

    public double getTempRight() {
        return m_right.getTemperature();
    }

    public void set(double speed) {
        m_left.set(speed);
        m_right.set(speed);
    }

    public double get(double speed) {
        return m_left.get();
    }

    public double getVelocity() {
        return m_left.getSelectedSensorVelocity();
    }

    public double getVelocityTarget() {
        return m_left.getClosedLoopTarget();
    }

    public void setVelocity(double velocity) {
        
        System.out.print("Desired: " + velocity + " ");
        double vel_now = m_left.getSelectedSensorVelocity();
        if (velocity < vel_now - 1000) {
            velocity = 0;
        }
        else if (velocity > vel_now + 1000) {
            velocity = vel_now + 1000;
        }
        System.out.println("Real: " + velocity + " Now: " + vel_now);
        if (velocity < 2000) {
            m_left.config_kI(0, 0);
            m_right.config_kI(0, 0);
            // compressor.enableDigital();
        } else {
            m_left.config_kI(0, 0.0001);
            m_right.config_kI(0, 0.0001);
            // compressor.disable();
        }
        m_left.set(TalonFXControlMode.Velocity, velocity);
        m_right.set(TalonFXControlMode.Velocity, velocity);
    }
}
