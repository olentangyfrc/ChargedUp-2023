// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.modules;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A swerve module using a Falcon 500 as drive motor, a NEO as angle motor, and an encoder for angle.
 * 
 * !!! WIP, needs to be tested. !!!
 * 
 */
public class SingleFalconModule extends SwerveModule {
    private static final double DRIVE_TICKS_PER_REVOLUTION = 2048;

    private double driveGearRatio;

    private WPI_TalonFX driveMotor;
    private CANSparkMax angleMotor;

    private AnalogInput angleEncoder;

    /**
     * Initialize a swerve module on the given ports
     * 
     * @param angleMotorChannel CAN port for the angle motor
     * @param driveMotorChannel CAN port for the drive motor
     * @param angleEncoderChannel Analog port for the angle encoder
     * @param driveGearRatio The number of times the actual wheel rotates for each rotation of the drive motor.
     * @param angleOffset The angle in degrees by which to offset the angle of the wheel.
     */
    public SingleFalconModule(int angleMotorChannel, int driveMotorChannel, int angleEncoderChannel, double angleOffset, double driveGearRatio, double maxSpeed) {
        angleMotor = new CANSparkMax(angleMotorChannel, MotorType.kBrushless);
        driveMotor = new WPI_TalonFX(driveMotorChannel);

        angleMotor.restoreFactoryDefaults();

        angleMotor.setInverted(true);

        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        angleEncoder = new AnalogInput(angleEncoderChannel);
        

        anglePid = new PIDController(0.5, 0, 0);
        anglePid.enableContinuousInput(0.0, 2 * Math.PI);
        anglePid.setTolerance(0.005);

        velocityFactorPID = new PIDController(0.2, 0, 0);
        velocityFactorPID.setSetpoint(0); // We want the error to be 0

        velocityConversionFactor = 3;
        velocityConversionOffset = 0.7;
        this.maxSpeed = maxSpeed;
        this.angleOffset = angleOffset;
        this.driveGearRatio = driveGearRatio;
    }

    /**
     * Set the percent output for the angle motor
     * 
     * @param output Percent output for the angle motor [-1, 1]
     */
    @Override
    public void setAnglePercentOutput(double output) {
        angleMotor.set(output);
    }

    /**
     * Get the angle of the swerve module
     * 
     * @return the angle as a rotation2D
     */
    @Override
    public Rotation2d getAngle() {
        // Raw angle
        double angle = (-angleEncoder.getValue() / ANGLE_ENCODER_TICKS * 2 * Math.PI); // Convert rotations to an angle in radians
        // Convert the offset into radians and subtract it from the angle
        angle -= angleOffset * Math.PI / 180;
        // Add a full rotation in radians to make sure the angle is always positive
        angle += 2 * Math.PI;
        // modulo the angle by a full rotation in radians to restrict it to the range [0,2Pi)
        angle %= 2 * Math.PI;
        return new Rotation2d(angle);
    }

    /**
     * Get the speed of the drive motor in meters per second
     * 
     * @return the speed of the drive motor in meters per second
     */
    @Override
    public double getVelocity() {
        return driveMotor.getSelectedSensorVelocity(0) * driveGearRatio / DRIVE_TICKS_PER_REVOLUTION * WHEEL_RADIUS * 2 * Math.PI * 10; // Multiply by 10 due to sample rate
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        SingleFalconModule other = (SingleFalconModule) obj;
        if (angleEncoder == null) {
            if (other.angleEncoder != null)
                return false;
        } else if (!angleEncoder.equals(other.angleEncoder))
            return false;
        if (angleMotor == null) {
            if (other.angleMotor != null)
                return false;
        } else if (!angleMotor.equals(other.angleMotor))
            return false;
        if (driveMotor == null) {
            if (other.driveMotor != null)
                return false;
        } else if (!driveMotor.equals(other.driveMotor))
            return false;
        return true;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition() * driveGearRatio / DRIVE_TICKS_PER_REVOLUTION * WHEEL_RADIUS * 2 * Math.PI, getAngle());
    }
    
}
