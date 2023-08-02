
package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {

    public static Object m_limelight;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");

    public static double x = tx.getDouble(0.0);
    public static double y = ty.getDouble(0.0);
    public static double area = ta.getDouble(0.0);

    public static boolean valid = false;

    public void update() {
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        // read values periodically
        double temp_x = tx.getDouble(0.0);
        double temp_y = ty.getDouble(0.0);
        double temp_area = ta.getDouble(0.0);

        if (temp_x == 0.0 && temp_y == 0.0 && temp_area == 0.0) {
            valid = false;
            return;
        } else {
            valid = true;
            x = x * 0.4 + temp_x * 0.6;
            y = y * 0.4 + temp_y * 0.6;
            area = area * 0.4 + temp_area * 0.6;
        }

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // // how many degrees back is your limelight rotated from perfectly vertical?
        // double limelightMountAngleDegrees = 25.0;

        // // distance from the center of the Limelight lens to the floor
        // double limelightLensHeightInches = 20.0;

        // // distance from the target to the floor
        // double goalHeightInches = 60.0;

        // double angleToGoalDegrees = limelightMountAngleDegrees +
        // targetOffsetAngle_Vertical;
        // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // // calculate distance
        // double distanceFromLimelightToGoalInches = (goalHeightInches -
        // limelightLensHeightInches)
        // / Math.tan(angleToGoalRadians);
        // SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);

    }

}
