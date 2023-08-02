
package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.swerve.Swerve;

public class Robot extends RobotBase {

    /* Joystick */
    public static XboxController joystick = new XboxController(0);
    public static final int joystick_axis_forward = 1;
    public static final int joystick_axis_sideway = 0;
    public static final int joystick_axis_rotation = 4;

    /* Sensors */
    public static AHRS AHRSSensor = new AHRS();
    public static LimeLight limelight = new LimeLight();
    public static ColorSensor color = new ColorSensor();

    /* Chassis */
    public static Swerve swerve = new Swerve(true);

    /* Motors */
    public static Climber climber = new Climber();
    public static Intake intake = new Intake();
    public static Shooter shooter = new Shooter();
    public static Aimer aimer = new Aimer();

    /* SB */
    private ShuffleboardTab sb_tab = Shuffleboard.getTab("Debug");
    private NetworkTableEntry sb_entry_shooting_target = sb_tab.add("Shooting Target", 10000).getEntry();
    private NetworkTableEntry sb_entry_shooting_angle = sb_tab.add("Shooting Angle", 0).getEntry();
    private NetworkTableEntry sb_entry_y = sb_tab.add("Limelight Y", 0).getEntry();

    public void robotInit() {

    }

    public void disabled() {
        while (!isEnabled()) {
            intake.m_down.setInverted(true);

            // System.out.println("LF: " + swerve.m_lf.getAbsoluteEncoderRad() + " " +
            // swerve.m_lf.getTurningPosition());
            // System.out.println("RF: " + swerve.m_rf.getAbsoluteEncoderRad() + " " +
            // swerve.m_rf.getTurningPosition());
            // System.out.println("LB: " + swerve.m_lb.getAbsoluteEncoderRad() + " " +
            // swerve.m_lb.getTurningPosition());
            // System.out.println("RB: " + swerve.m_rb.getAbsoluteEncoderRad() + " " +
            // swerve.m_rb.getTurningPosition());

            // System.out.println(swerve.m_rf.getAbsoluteEncoderRad());
            

            // System.out.println(Intake.m_down.getInverted() + " " + swerve.m_lf.m_driveMotor.getInverted() + " " + swerve.m_rf.m_driveMotor.getInverted() + " " + swerve.m_lb.m_driveMotor.getInverted() + " " + swerve.m_rb.m_driveMotor.getInverted());

            Timer.delay(1);
        }
    }

    public double aiming_speed(double y) {
        return y = 0.0024*y*y*y*y - 0.1904*y*y*y + 2.8998*y*y - 25.566*y + 9480;
    }

    public double aiming_angle(double y) {
        return -0.1 * y + 4;
    }

    public void autonomous() {
        // isDisabled() need to be called
    }

    public boolean ready_shoot(double target) {
        return (Math.abs(target - shooter.getVelocity()) < 20);
    }

    public void teleop() {

        climber.setSoftLimit(true);
        aimer.setSoftLimit(true);

        boolean intakeOuting = false;
        double intakeOutingDDL = 0;

        double shooting_target = 10000;
        double shooting_angle = 4.5;
        boolean shooting = false;
        boolean aiming = false;

        PIDController pid_aimer = new PIDController(0.12, 0, 0.002, 0.005);
        pid_aimer.setSetpoint(0);

        while (isTeleop() && isEnabled()) {

            /* Drivetrain chassis */

            double joystick_forward = -MathUtil.applyDeadband(joystick.getRawAxis(joystick_axis_forward), 0.04);
            double joystick_sideway = MathUtil.applyDeadband(joystick.getRawAxis(joystick_axis_sideway), 0.04);
            double joystick_rotation = MathUtil.applyDeadband(joystick.getRawAxis(joystick_axis_rotation), 0.04);

            if (!joystick.getRawButton(5)) {
                swerve.drive(joystick_forward * 2.2, joystick_sideway * 2.2,
                        joystick_rotation * 3); // 3.5 3.5 7
            }

            /* Intake */

            if (!intakeOuting) {
                if (false) {//color.what() == ColorSensor.ColorResult.Red) {
                    intakeOuting = true;
                    intakeOutingDDL = Timer.getFPGATimestamp() + 1;
                    continue;
                } else {
                    if (joystick.getRawAxis(3) > 0.15) {
                        intake.set_solenoid(true);
                        intake.set_eat(0.6);
                        intake.set_down(1);
                    } else if (joystick.getRawButton(6)) {
                        intake.set_solenoid(false);
                        intake.set_eat(-0.6);
                        intake.set_down(-1);
                    } else if (shooting) {
                        // TODO: Shooting intake logic
                        if (ready_shoot(shooting_target)) {
                            intake.set_eat(0);
                            intake.set_down(1);
                        } else {
                            intake.set_down(0);
                            intake.set_down(0);
                        }
                    } else {
                        intake.set_solenoid(false);
                        intake.set_eat(0);
                        intake.set_down(0);
                    }

                    if (joystick.getRawButton(6)) {
                        intake.set_up(-1);
                    } else if (shooting) {
                        if (ready_shoot(shooting_target)) {
                            intake.set_up(1);
                        } else {
                            intake.set_up(0);
                        }
                    } else {
                        intake.set_up(0);
                    }

                }
            } else {
                intake.set_solenoid(false);
                intake.set_down(-1);
                if (Timer.getFPGATimestamp() > intakeOutingDDL) {
                    intakeOuting = false;
                }
            }

            /* Climber (Threshold : -8000 ~ -421000) */

            if (joystick.getYButton() && climber.getPosition() > -421000) {
                climber.set(-1);
            } else if (joystick.getAButton() && climber.getPosition() < -8000) {
                climber.set(1);
            } else {
                if (climber.getPosition() <= -421000) {
                    climber.set(0.15);
                } else {
                    climber.set(0);
                }
            }

            /* Shooter */

            if (joystick.getRawAxis(2) > 0.15) {
                shooter.setVelocity(shooting_target);
                // TODO: shooting_target = shooting_target;
                shooting = true;
            } else {
                shooting = false;
                shooter.setVelocity(1000);
                // shooter.setVelocity(joystick_forward*10000);
                // Shooter.m_left.set(joystick_forward);
                // Shooter.m_right.set(-joystick.getRawAxis(5));
            }
            

            if (joystick.getRawButton(5)) {
                aiming = true;
                limelight.update();
                if (limelight.valid) {
                    double aimer_turn = Math.min(0.4, pid_aimer.calculate(-limelight.x));
                    swerve.drive(joystick_forward * 1.4, joystick_sideway * 1.4, aimer_turn);
                } else {
                    swerve.drive(joystick_forward * 2.2, joystick_sideway * 2.2,
                            joystick_rotation * 3); // 3.5 3.5 7
                }
            } else {
                aiming = shooting;
                pid_aimer.calculate(0);
            }

            /* Aimer */

            // aimer.set(joystick_forward * 0.2);

            // if (joystick.getRawButton(7)) {
            // aimer.setPosition(2);
            // } else if (joystick.getRawButton(8)) {
            // aimer.setPosition(4.5);
            // }

            aimer.setPosition(shooting_angle);

            System.out.println("Shooter Temp: " + shooter.getTempLeft() + " " + shooter.getTempRight());

            

            // System.out.println(aimer.get() + " " + aimer.getPosition() + " " +
            // aimer.getCurrent());

            /* Debug */

            // System.out.println("LF: " + swerve.m_lf.getDrivePosition() + " " +
            // swerve.m_lf.getTurningPosition());
            // System.out.println("RF: " + swerve.m_rf.getDrivePosition() + " " +
            // swerve.m_rf.getTurningPosition());
            // System.out.println("LB: " + swerve.m_lb.getDrivePosition() + " " +
            // swerve.m_lb.getTurningPosition());
            // System.out.println("RB: " + swerve.m_rb.getDrivePosition() + " " +
            // swerve.m_rb.getTurningPosition());

            // System.out.println(aimer.getPosition());

            limelight.update();

            if (LimeLight.valid) {
                shooting_target = shooting_target * 0.8 + aiming_speed(limelight.y) * 0.2;
                shooting_angle = shooting_angle * 0.8 + aiming_angle(limelight.y) * 0.2;
                sb_entry_y.setDouble(limelight.y);
                // shooting_target = sb_entry_shooting_target.getDouble(8000);
                sb_entry_shooting_angle.setDouble(shooting_angle);
                sb_entry_shooting_target.setDouble(shooting_target);
                // System.out.println(limelight.y + " " + shooting_target + " " + shooting_angle);
            }



            Timer.delay(0.005);
        }

        stopAllMotors();
    }

    public void test() {
        // Init robot
        // 1. climber down
        // 2. aimer down
        // 3. intake solenoid allow move
        // 4. standard teleop program
        aimer.setSoftLimit(false);
        aimer.set(0);
        {
            // Climber reset
            while (!joystick.getBButton()) {
                Timer.delay(0.1);
            }
            climber.setSoftLimit(false);
            climber.set(0.2);
            while (climber.getCurrent() < 15.0) {
                Timer.delay(0.02);
            }
            climber.set(0);
            Timer.delay(0.2);
            climber.reset();
            climber.setSoftLimit(true);
            Climber.inited_outer = true;
        }
        {
            // Aimer reset

            while (!joystick.getAButton()) {
                Timer.delay(0.1);
            }
            while (joystick.getAButton()) {
                // System.out.println(aimer.getPosition());
                Timer.delay(0.1);
            }
            aimer.reset();
            aimer.setSoftLimit(true);
            Aimer.inited_outer = true;

        }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    private volatile boolean m_exit;

    @Override
    public void startCompetition() {
        stopAllMotors();
        robotInit();
        stopAllMotors();

        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();

        while (!Thread.currentThread().isInterrupted() && !m_exit) {
            if (isDisabled()) {
                DriverStation.inDisabled(true);
                stopAllMotors();
                disabled();
                stopAllMotors();
                DriverStation.inDisabled(false);
                while (isDisabled()) {
                    DriverStation.waitForData();
                }
            } else if (isAutonomous()) {
                DriverStation.inAutonomous(true);
                stopAllMotors();
                autonomous();
                stopAllMotors();
                DriverStation.inAutonomous(false);
                while (isAutonomousEnabled()) {
                    DriverStation.waitForData();
                }
            } else if (isTest()) {
                // LiveWindow.setEnabled(true);
                // Shuffleboard.enableActuatorWidgets();
                DriverStation.inTest(true);
                stopAllMotors();
                test();
                stopAllMotors();
                DriverStation.inTest(false);
                while (isTest() && isEnabled()) {
                    DriverStation.waitForData();
                }
                // LiveWindow.setEnabled(false);
                // Shuffleboard.disableActuatorWidgets();
            } else {
                DriverStation.inTeleop(true);
                stopAllMotors();
                teleop();
                stopAllMotors();
                DriverStation.inTeleop(false);
                while (isTeleopEnabled()) {
                    DriverStation.waitForData();
                }
            }
        }
        stopAllMotors();
    }

    @Override
    public void endCompetition() {
        m_exit = true;
        stopAllMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void stopAllMotors() {
        swerve.m_lb.stop();
        swerve.m_lf.stop();
        swerve.m_rb.stop();
        swerve.m_rf.stop();
        climber.set(0);
        intake.set_down(0);
        intake.set_up(0);
        intake.set_eat(0);
        shooter.set(0);
        resetAllCylinders();
    }

    private void resetAllCylinders() {
        intake.set_solenoid(false);
    }
}
