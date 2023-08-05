
package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.ColorSensor.ColorResult;
import frc.robot.swerve.Swerve;

public class Robot extends RobotBase {

    /* Joystick */
    public static XboxController joystick = new XboxController(0);
    public static final int joystick_axis_forward = 1;
    public static final int joystick_axis_sideway = 0;
    public static final int joystick_axis_rotation = 4;

    /* Sensors */
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
    private ShuffleboardTab sb_tab = Shuffleboard.getTab("Dashboard");
    private NetworkTableEntry sb_entry_shooting_target = sb_tab.add("Shooting Target", 10000).getEntry();
    private NetworkTableEntry sb_entry_shooting_angle = sb_tab.add("Shooting Angle", 0).getEntry();
    private NetworkTableEntry sb_entry_y = sb_tab.add("Limelight Y", 0).getEntry();

    private NetworkTableEntry sb_odo_x = sb_tab.add("Odo X", 0).getEntry();
    private NetworkTableEntry sb_odo_y = sb_tab.add("Odo Y", 0).getEntry();
    private NetworkTableEntry sb_odo_t = sb_tab.add("Odo T", 0).getEntry();

    private NetworkTableEntry sb_auto_chooser = sb_tab.add("Autonomous Program", "0").getEntry();
    private NetworkTableEntry sb_color_chooser = sb_tab.add("Color", "Blue").getEntry();

    public void robotInit() {
        sb_auto_chooser.setString("3");
        sb_color_chooser.setString("Blue");
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
            // System.out.println("Climber: " + climber.getPosition());

            String color_chosen = sb_color_chooser.getString("Blue");
            ColorResult color_opposite;
            if (color_chosen == "Blue") {
                color_opposite = ColorResult.Red;
            } else {
                color_opposite = ColorResult.Blue;
            }
            // System.out.println("Color: " + color_chosen + " " +
            // (color_chosen.equals("Blue")));
        }
    }

    public double aiming_speed(double y) {
        return y = 0.0024 * y * y * y * y - 0.1904 * y * y * y + 2.8998 * y * y - 25.566 * y + 9480;
    }

    public double aiming_angle(double y) {
        return -0.1 * y + 4;
    }

    public void practice() {

    }

    public void autonomous() {
        String auto_chosen = sb_auto_chooser.getString("1");
        switch (auto_chosen) {
            // case "0": {

            // // isDisabled() need to be called
            // PIDController pid_x = new PIDController(5, 0, 1);
            // PIDController pid_y = new PIDController(5, 0, 1);
            // PIDController pid_t = new PIDController(0.06, 0, 0);

            // double maxSpeed = 0.8;
            // double maxTurn = 2.4;

            // double time = Timer.getFPGATimestamp();

            // swerve.ahrs.zeroYaw();
            // swerve.updateOdometry();
            // swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new
            // Rotation2d(0));
            // swerve.updateOdometry();

            // pid_x.setSetpoint(1);
            // pid_y.setSetpoint(0);
            // pid_t.setSetpoint(0);

            // intake.set_eat(1);
            // intake.set_solenoid(false);

            // while (Timer.getFPGATimestamp() - time < 0.8 && isAutonomous() &&
            // isEnabled()) {

            // swerve.drive(
            // Math.min(Math.max(-maxSpeed,
            // pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
            // maxSpeed),
            // Math.min(Math.max(-maxSpeed,
            // pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
            // maxSpeed),
            // Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())),
            // maxTurn),
            // true);
            // swerve.updateOdometry();

            // if (swerve.m_odometry.getPoseMeters().getX() > 0.5) {
            // intake.set_down(1);
            // intake.set_solenoid(true);
            // }

            // if (Math.abs(swerve.m_odometry.getPoseMeters().getX() - 1) > 0.05) {
            // time = Timer.getFPGATimestamp();
            // }

            // Timer.delay(0.005);
            // }
            // time = Timer.getFPGATimestamp();

            // pid_x.setSetpoint(1);
            // pid_y.setSetpoint(0);
            // pid_t.setSetpoint(180);

            // intake.set_eat(0);
            // intake.set_down(0);
            // intake.set_solenoid(false);

            // maxSpeed = 1.8;
            // maxTurn = 2.4;

            // time = Timer.getFPGATimestamp();

            // while (Timer.getFPGATimestamp() - time < 0.4 && isAutonomous() &&
            // isEnabled()) {

            // shooter.setVelocity(10000);
            // aimer.setPosition(3);

            // swerve.drive(
            // Math.min(Math.max(-maxSpeed,
            // pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
            // maxSpeed),
            // Math.min(Math.max(-maxSpeed,
            // pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
            // maxSpeed),
            // Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())),
            // maxTurn),
            // true);
            // swerve.updateOdometry();

            // if (Math.abs(swerve.ahrs.getAngle() - 175) > 5) {
            // time = Timer.getFPGATimestamp();
            // }

            // Timer.delay(0.005);
            // }
            // time = Timer.getFPGATimestamp();
            // swerve.drive(0, 0, 0);

            // // Shoot ball

            // intake.set_up(0.5);
            // intake.set_down(0.5);

            // time = Timer.getFPGATimestamp();

            // while (Timer.getFPGATimestamp() - time < 3) {
            // swerve.updateOdometry();
            // Timer.delay(0.005);
            // }
            // }
            // break;
            case "0": {
                PIDController pid_aimer = new PIDController(0.12, 0, 0.002, 0.005);
                double shooting_angle = 2.5;
                double shooting_target = 9227;
                double time = Timer.getFPGATimestamp();
                while (isAutonomous() && isEnabled()) {
                    limelight.update();
                    if (limelight.valid) {
                        double aimer_turn = Math.min(0.4, Math.max(-0.4, pid_aimer.calculate(-limelight.x)));
                        swerve.drive(0, 0, aimer_turn);
                        shooting_target = shooting_target * 0.8 + aiming_speed(limelight.y) * 0.2;
                        shooting_angle = shooting_angle * 0.8 + aiming_angle(limelight.y) * 0.2;
                    }
                    shooter.setVelocity(shooting_target);
                    if (!ready_shoot(shooting_target)) {
                        time = Timer.getFPGATimestamp();
                        intake.set_up(0);
                    } else {
                        if (Timer.getFPGATimestamp() - time > 0.5) {
                            intake.set_up(1);
                        }
                        else {
                            intake.set_up(0);
                        }
                    }
                    swerve.updateOdometry();
                    Timer.delay(0.005);
                }

            }
            case "2": {

                // isDisabled() need to be called
                PIDController pid_x = new PIDController(5, 0, 1);
                PIDController pid_y = new PIDController(5, 0, 1);
                PIDController pid_t = new PIDController(0.06, 0, 0);

                double maxSpeed = 0.8;
                double maxTurn = 2.4;

                double time = Timer.getFPGATimestamp();

                swerve.ahrs.zeroYaw();
                swerve.updateOdometry();
                swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new Rotation2d(0));
                swerve.updateOdometry();

                pid_x.setSetpoint(1);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(0);

                intake.set_eat(1);
                intake.set_solenoid(false);

                while (Timer.getFPGATimestamp() - time < 0.8 && isAutonomous() && isEnabled()) {

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (swerve.m_odometry.getPoseMeters().getX() > 0.5) {
                        intake.set_down(1);
                        intake.set_solenoid(true);
                    }

                    if (Math.abs(swerve.m_odometry.getPoseMeters().getX() - 1) > 0.05) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();

                pid_x.setSetpoint(1);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(175);

                intake.set_eat(0);
                intake.set_down(0);
                intake.set_solenoid(false);

                maxSpeed = 1.8;
                maxTurn = 2.4;

                time = Timer.getFPGATimestamp();

                while (Timer.getFPGATimestamp() - time < 0.4 && isAutonomous() && isEnabled()) {

                    shooter.setVelocity(10000);
                    aimer.setPosition(3);

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (Math.abs(swerve.ahrs.getAngle() - 175) > 5) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();
                swerve.drive(0, 0, 0);

                // Shoot ball

                intake.set_up(0.5);
                intake.set_down(0.5);

                time = Timer.getFPGATimestamp();

                while (Timer.getFPGATimestamp() - time < 3) {
                    swerve.updateOdometry();
                    Timer.delay(0.005);
                }
            }
                break;
            case "1": {

                // isDisabled() need to be called
                PIDController pid_x = new PIDController(5, 0, 1);
                PIDController pid_y = new PIDController(5, 0, 1);
                PIDController pid_t = new PIDController(0.06, 0, 0);

                double maxSpeed = 1;// 1;
                double maxTurn = 1.5;// 2.4;

                double time = Timer.getFPGATimestamp();

                swerve.ahrs.zeroYaw();
                swerve.updateOdometry();
                swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new Rotation2d(0));
                swerve.updateOdometry();

                pid_x.setSetpoint(0.85);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(0);

                intake.set_eat(1);
                intake.set_solenoid(false);

                while (Timer.getFPGATimestamp() - time < 0.8 && isAutonomous() && isEnabled()) {

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (swerve.m_odometry.getPoseMeters().getX() > 0) {
                        intake.set_down(1);
                        intake.set_solenoid(true);
                    }

                    if (Math.abs(swerve.m_odometry.getPoseMeters().getX() - 0.85) > 0.05) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();

                maxSpeed = 1.8;
                maxTurn = 2;

                pid_x.setSetpoint(0.0);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(-155);

                intake.set_eat(0);
                intake.set_down(0);
                intake.set_up(-0.2);
                intake.set_solenoid(false);

                while (Timer.getFPGATimestamp() - time < 0.8 && isAutonomous() && isEnabled()) {

                    shooter.setVelocity(9600);
                    aimer.setPosition(3);

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (Math.abs(swerve.m_odometry.getPoseMeters().getX() - 0) > 0.05
                            || Math.abs(swerve.ahrs.getAngle() - (-155)) > 5) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();
                swerve.drive(0, 0, 0);

                // Shoot ball

                intake.set_up(0.5);
                intake.set_down(0.5);

                time = Timer.getFPGATimestamp();

                while (Timer.getFPGATimestamp() - time < 3) {
                    swerve.updateOdometry();
                    Timer.delay(0.005);
                }
            }
                break;
            case "3": {

                // isDisabled() need to be called
                PIDController pid_x = new PIDController(5, 0, 1);
                PIDController pid_y = new PIDController(5, 0, 1);
                PIDController pid_t = new PIDController(0.06, 0, 0);

                double maxSpeed = 0.8;
                double maxTurn = 2.4;

                double time = Timer.getFPGATimestamp();

                swerve.ahrs.zeroYaw();
                swerve.updateOdometry();
                swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new Rotation2d(0));
                swerve.updateOdometry();

                pid_x.setSetpoint(1.15);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(0);

                intake.set_eat(1);
                intake.set_solenoid(false);

                while (Timer.getFPGATimestamp() - time < 0.8 && isAutonomous() && isEnabled()) {

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (swerve.m_odometry.getPoseMeters().getX() > 0.5) {
                        intake.set_down(1);
                        intake.set_solenoid(true);
                    }

                    if (Math.abs(swerve.m_odometry.getPoseMeters().getX() - 1.15) > 0.05) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();

                pid_x.setSetpoint(1.15);
                pid_y.setSetpoint(0);
                pid_t.setSetpoint(160);

                intake.set_eat(0);
                intake.set_down(0);
                intake.set_solenoid(false);

                maxSpeed = 1.8;
                maxTurn = 2.4;

                time = Timer.getFPGATimestamp();

                while (Timer.getFPGATimestamp() - time < 0.4 && isAutonomous() && isEnabled()) {

                    shooter.setVelocity(10000);
                    aimer.setPosition(3);

                    swerve.drive(
                            Math.min(Math.max(-maxSpeed, pid_x.calculate(swerve.m_odometry.getPoseMeters().getX())),
                                    maxSpeed),
                            Math.min(Math.max(-maxSpeed, pid_y.calculate(swerve.m_odometry.getPoseMeters().getY())),
                                    maxSpeed),
                            Math.min(Math.max(-maxTurn, pid_t.calculate(swerve.ahrs.getAngle())), maxTurn),
                            true);
                    swerve.updateOdometry();

                    if (Math.abs(swerve.ahrs.getAngle() - 160) > 5) {
                        time = Timer.getFPGATimestamp();
                    }

                    Timer.delay(0.005);
                }
                time = Timer.getFPGATimestamp();
                swerve.drive(0, 0, 0);

                // Shoot ball

                intake.set_up(0.5);
                intake.set_down(0.5);

                time = Timer.getFPGATimestamp();

                while (Timer.getFPGATimestamp() - time < 3) {
                    swerve.updateOdometry();
                    Timer.delay(0.005);
                }
            }
                break;
        }

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

            String color_chosen = sb_color_chooser.getString("Blue");
            ColorResult color_opposite;
            if (color_chosen.equals("Blue")) {
                color_opposite = ColorResult.Red;
            } else if (color_chosen.equals("Red")) {
                color_opposite = ColorResult.Blue;
            } else {
                color_opposite = ColorResult.None;
            }

            if (!intakeOuting) {
                if (color_opposite != ColorResult.None && color.what() == color_opposite) {
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

            if (joystick.getYButton() && climber.getPosition() > -415562) {
                climber.set(-1);
            } else if (joystick.getAButton() && climber.getPosition() < -8000) {
                climber.set(1);
            } else {
                if (climber.getPosition() <= -415562) {
                    climber.set(0.05);
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
                if (joystick.getRawButton(7)) {
                    shooter.set(1);
                } else if (joystick.getRawButton(8)) {
                    shooter.set(-1);
                } else {
                    shooter.set(0);
                }
                // shooter.setVelocity(joystick_forward*10000);
                // Shooter.m_left.set(joystick_forward);
                // Shooter.m_right.set(-joystick.getRawAxis(5));
            }

            if (joystick.getRawButton(5)) {
                aiming = true;
                limelight.update();
                if (limelight.valid) {
                    double aimer_turn = Math.min(0.4, Math.max(-0.4, pid_aimer.calculate(-limelight.x)));
                    swerve.drive(joystick_forward * 1.4, joystick_sideway * 1.4, aimer_turn);
                } else {
                    swerve.drive(joystick_forward * 2.2, joystick_sideway * 2.2,
                            joystick_rotation * 3); // 3.5 3.5 7
                }
            } else {
                aiming = shooting;
                pid_aimer.calculate(0);
            }

            // if (joystick.getRawButton(7)) {
            // swerve.ahrs.zeroYaw();
            // swerve.updateOdometry();
            // swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new
            // Rotation2d(0));
            // swerve.updateOdometry();
            // }

            /* Aimer */

            // aimer.set(joystick_forward * 0.2);

            // if (joystick.getRawButton(7)) {
            // aimer.setPosition(2);
            // } else if (joystick.getRawButton(8)) {
            // aimer.setPosition(4.5);
            // }

            aimer.setPosition(shooting_angle);

            // System.out.println("Shooter Temp: " + shooter.getTempLeft() + " " +
            // shooter.getTempRight());

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
            swerve.updateOdometry();

            // System.out.println("Odometry: " + swerve.m_odometry.getPoseMeters());
            sb_odo_t.setDouble(swerve.m_odometry.getPoseMeters().getRotation().getDegrees());
            sb_odo_x.setDouble(swerve.m_odometry.getPoseMeters().getX());
            sb_odo_y.setDouble(swerve.m_odometry.getPoseMeters().getY());

            if (LimeLight.valid) {
                shooting_target = shooting_target * 0.8 + aiming_speed(limelight.y) * 0.2;
                shooting_angle = shooting_angle * 0.8 + aiming_angle(limelight.y) * 0.2;
                sb_entry_y.setDouble(limelight.y);
                // shooting_target = sb_entry_shooting_target.getDouble(8000);
                sb_entry_shooting_angle.setDouble(shooting_angle);
                sb_entry_shooting_target.setDouble(shooting_target);
                // System.out.println(limelight.y + " " + shooting_target + " " +
                // shooting_angle);
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
        {
            swerve.ahrs.zeroYaw();
            swerve.updateOdometry();
            swerve.m_odometry.resetPosition(new Pose2d(0., 0., new Rotation2d(0)), new Rotation2d(0));
            swerve.updateOdometry();
        }
        while (isEnabled()) {
            // System.out.println("Climber: " + climber.getPosition());
            if (joystick.getYButton() && climber.getPosition() > -415562) {
                climber.set(-1);
            } else if (joystick.getAButton() && climber.getPosition() < -8000) {
                climber.set(1);
            } else {
                if (climber.getPosition() <= -415562) {
                    climber.set(0.05);
                } else {
                    climber.set(0);
                }
            }
            Timer.delay(0.005);
        }

        // while (isEnabled() && isTest()) {

        // shooter.m_left.set(-joystick.getRawAxis(1));
        // shooter.m_right.set(-joystick.getRawAxis(5));

        // Timer.delay(0.005);
        // }

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
        Shooter.compressor.enableDigital();
        resetAllCylinders();
    }

    private void resetAllCylinders() {
        intake.set_solenoid(false);
    }
}
