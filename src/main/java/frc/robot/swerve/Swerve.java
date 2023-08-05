
package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

public class Swerve {

    // TODO: Update speed limits
    public static final double kMaxSpeed = 5.0; // 3 meters per second
    public static final double kMaxAngularSpeed = 4 * Math.PI; // 1/2 rotation per second

    public static final double k_lf_abs_enc_offset = 3.962054;
    public static final double k_rf_abs_enc_offset = 5.113449829316348;
    public static final double k_lb_abs_enc_offset = 1.5226087;
    public static final double k_rb_abs_enc_offset = 2.401198196463655;

    // public static final double k_lf_abs_enc_offset = 0;
    // public static final double k_rf_abs_enc_offset = 0;
    // public static final double k_lb_abs_enc_offset = 0;
    // public static final double k_rb_abs_enc_offset = 0;

    public final SwerveModule m_lf = new SwerveModule(2, 1, false, true, 0, k_lf_abs_enc_offset, false, 1);
    public final SwerveModule m_rf = new SwerveModule(3, 4, true, true, 1, k_rf_abs_enc_offset, false, 1);
    public final SwerveModule m_lb = new SwerveModule(7, 8, false, true, 3, k_lb_abs_enc_offset, false, 1);
    public final SwerveModule m_rb = new SwerveModule(5, 6, true, true, 2, k_rb_abs_enc_offset, false, 1);

    public final AHRS ahrs = new AHRS();

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(0.26, -0.26), // LF
            new Translation2d(0.26, 0.26), // RF
            new Translation2d(-0.26, -0.26), // L   B
            new Translation2d(-0.26, 0.26)); // RB

    public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, ahrs.getRotation2d());

    public Swerve(boolean resetHeading) {
        if (resetHeading)
            ahrs.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed   Speed of the robot in the x direction (forward).
     * @param ySpeed   Speed of the robot in the y direction (sideways).
     * @param rot      Angular rate of the robot.
     * @param headless Whether the provided x and y speeds are relative to the
     *                 field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean headless) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                headless
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_lf.setDesiredState(swerveModuleStates[0]);
        m_rf.setDesiredState(swerveModuleStates[1]);
        m_lb.setDesiredState(swerveModuleStates[2]);
        m_rb.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Method to drive the robot using joystick info, with headless disabled as
     * default.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_lf.setDesiredState(swerveModuleStates[0]);
        m_rf.setDesiredState(swerveModuleStates[1]);
        m_lb.setDesiredState(swerveModuleStates[2]);
        m_rb.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the field relative position of the robot,
     * Should be called periodically during chassis movement.
     */
    public void updateOdometry() {
        m_odometry.update(
                ahrs.getRotation2d(),
                m_lf.getState(),
                m_rf.getState(),
                m_lb.getState(),
                m_rb.getState());
    }
}
