package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {
    public static CANSparkMax m_eat = new CANSparkMax(12, MotorType.kBrushless);
    public static CANSparkMax m_down = new CANSparkMax(9, MotorType.kBrushless);
    public static CANSparkMax m_up = new CANSparkMax(10, MotorType.kBrushless);
    public static DigitalInput optic_sensor = new DigitalInput(0);

    private final ColorSensorV3 color_sensor = new ColorSensorV3(I2C.Port.kOnboard);

    public static DoubleSolenoid s_eat = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    public static Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    public static boolean inited = false;

    public Intake() {

            m_down.restoreFactoryDefaults();
            m_up.restoreFactoryDefaults();
            m_eat.restoreFactoryDefaults();
            
            m_down.setInverted(true);
            m_up.setInverted(true);
            m_eat.setInverted(false);

            

            inited = true;
    }

    public void set_eat(double speed) {
        m_eat.set(speed);
    }

    public void set_up(double speed) {
        m_up.set(speed);
    }

    public void set_down(double speed) {
        m_down.set(speed);
    }

    public double get_eat() {
        return m_eat.get();
    }

    public double get_up() {
        return m_up.get();
    }

    public double get_down() {
        return m_down.get();
    }

    public boolean get_optic() {
        return !optic_sensor.get();
    }

    public void set_solenoid(boolean down) {
        if (down) {
            s_eat.set(Value.kForward);
        }
        else {
            s_eat.set(Value.kReverse);
        }
    }

    

}
