// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveModule {

    /* Constants */
    private static final double kModuleMaxAngularVelocity = Swerve.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 3 * Math.PI; // radians per second squared

    private static int id_cnt = 0;
    private final int id;

    /* Motors */
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    /* Abs Encoder */
    public final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    // TODO: Gains are for example purposes only - must be determined for your own
    // robot!
    private final SparkMaxPIDController m_drivePIDController; // 0.3
    private final SparkMaxPIDController m_turnPIDController; // 0.3

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderChannel,
            double absoluteEncoderOffset,
            boolean absoluteEncoderReversed,
            double wheelSizeCompensationRatio) {

        id = id_cnt;
        id_cnt++;

        this.absoluteEncoder = new AnalogInput(absoluteEncoderChannel);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor.restoreFactoryDefaults();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setP(0.01);
        m_drivePIDController.setI(0.002);
        m_drivePIDController.setD(0);
        m_drivePIDController.setIZone(0.3);
        m_drivePIDController.setFF(0.4);
        m_drivePIDController.setOutputRange(-1, 1);

        m_turnPIDController = m_turningMotor.getPIDController();
        m_turnPIDController.setP(1);
        m_turnPIDController.setI(1e-4);
        m_turnPIDController.setD(1);
        m_turnPIDController.setIZone(0.5);
        m_turnPIDController.setFF(0);
        m_turnPIDController.setOutputRange(-1, 1);

        m_driveMotor.setInverted(driveMotorReversed);
        m_turningMotor.setInverted(turningMotorReversed);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setPositionConversionFactor(0.026868672416183105 / wheelSizeCompensationRatio);
        m_driveEncoder.setVelocityConversionFactor(4.478112069363851E-4 / wheelSizeCompensationRatio);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.setPositionConversionFactor(0.3490658503988659);
        m_turningEncoder.setVelocityConversionFactor(0.005817764173314432);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        // m_turnPIDController.setPositionPIDWrappingEnabled(true);
        // m_turnPIDController.setPositionPIDWrappingMaxInput(Math.PI);
        // m_turnPIDController.setPositionPIDWrappingMinInput(-Math.PI);
        // m_turningPIDController.setIntegratorRange(-20, 20);

        resetEncoders();
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return m_turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return m_turningEncoder.getVelocity();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    private static double modulous(double a, double b) {
        return (a % b + b) % b;
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        // desiredState: -PI ~ PI,
        // currentAngle: continuous,
        var currentAngle_remainder = modulous(currentAngle.getDegrees() + 180, 360) - 180;
        var currentAngle_offset = currentAngle.getDegrees() - currentAngle_remainder;
        var delta = desiredState.angle.getDegrees() - currentAngle_remainder;
        if (Math.abs(delta) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle
                            .rotateBy(Rotation2d.fromDegrees(currentAngle_remainder))
                            .rotateBy(Rotation2d.fromDegrees(180.0 * Math.signum(delta))));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(currentAngle_remainder)));
        }
    }

    private int encoderWindUp = 0;
    private double prevAngle = 0.0;

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.02) {
            desiredState.speedMetersPerSecond = 0;
        }

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningEncoder.getPosition()));

        System.out.println(state.speedMetersPerSecond + " " + m_driveEncoder.getVelocity());

        m_drivePIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // New Code added to handle the 180 to -180 and -180 t0 180 boundary condition
        if (prevAngle > 90) {
            // Our previous angle was in an area where a wrap could oocur (SW Corner)
            if (state.angle.getDegrees() < -90) {
                // If we are here it means that target setpoint needs to be adjusted to prevent
                // the
                // module from taking the long way to reach the target in the SE Corner.
                encoderWindUp++;
            }
        }


        if (prevAngle < -90) {
            // Our previous angle was in an area where a wrap could occur (SE Corner)
            if (state.angle.getDegrees() > 90) {
                // If we are here it mean that target setpoint needs to be adjusted to prevent
                // the
                // module from taking the long way to reach the target in the SW Corner.
                encoderWindUp--;
            }
        }

        // Update the last setpoint to the current setpoint
        prevAngle = state.angle.getDegrees();
        
        m_turnPIDController.setReference(state.angle.getRadians() + encoderWindUp * 2 * Math.PI, CANSparkMax.ControlType.kPosition);

    }

    public void stop() {
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }
}
