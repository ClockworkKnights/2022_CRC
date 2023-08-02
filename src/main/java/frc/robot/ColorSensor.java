package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    public enum ColorResult {
        None,
        Red,
        Blue
    }

    public ColorSensor() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public ColorResult what() {
        Color detectedColor = m_colorSensor.getColor();
        double detectedProximity = m_colorSensor.getProximity() / 2048.0; // 0 to 2047
        // double IR = m_colorSensor.getIR();

        // SmartDashboard.putNumber("Red", detectedColor.red);
        // SmartDashboard.putNumber("Green", detectedColor.green);
        // SmartDashboard.putNumber("Blue", detectedColor.blue);
        // SmartDashboard.putNumber("IR", IR);

        if (detectedProximity < 0.3) { // TODO: Change proximity threshold
            return ColorResult.None;
        }

        System.out.println("Color proximity: " + detectedProximity);

        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            return ColorResult.Blue;
        } else if (match.color == kRedTarget) {
            return ColorResult.Red;
        }

        return ColorResult.None;
    }
}
