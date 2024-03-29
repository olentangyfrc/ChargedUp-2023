package frc.robot.subsystems.drivetrain;

import java.util.logging.Logger;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import frc.robot.telemetry.OzoneImu;

public abstract class SwerveDrivetrain extends SubsystemBase {

    // Declaring Swerve Modules
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    // Distance from center of wheel to center of wheel across the side of the bot in meters
    public static final double WHEEL_BASE = 0.496;
    // Distance from center of wheel to center of wheel across the front of the bot in meters
    public static final double TRACK_WIDTH = 0.547;

    
    public static final double MAX_LINEAR_SPEED = 3; // Meters per second
    public static final double MAX_LINEAR_ACCELERATION = 2.5; // Meters per second squared
    // public static final double MAX_LINEAR_SPEED = 6; // Meters per second
    // public static final double MAX_LINEAR_ACCELERATION = 12; // Meters per second squared

    public static final double MAX_ROTATION_SPEED = 15; // Radians per second
    public static final double MAX_ROTATION_ACCELERATION = 8 * 2 * Math.PI; // Radians per second squared

    // In meters per second
    public static final double IS_MOVING_TRANSLATION_TOLERANCE = 3.5;
    // In radians per second
    public static final double IS_MOVING_ROTATION_TOLERANCE = 1;


    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;

    public PIDConstants translationPidConstants;
    public PIDConstants rotationPidConstants;


    // Used to convert from ChassisSpeeds to SwerveModuleStates
    private SwerveDriveKinematics kinematics;

    private Logger logger = Logger.getLogger("DrivetrainSubsystem");
    
    // Odometry
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();
    
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private GenericEntry fieldOrientedToggle = tab.add("Field Oriented", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private PIDController anglePid = new PIDController(.12, 0, 0);
    private double targetAngle = Double.NaN;
    private boolean isAtTargetAngle = false;

    private boolean isInBrakeMode = false;
    private boolean isFollowingPath = false;

    private double desiredSpeedPercent = 1;
    private double speedPercent = desiredSpeedPercent;
    public static final double DEFAULT_SPEED_PERCENT = 1;
    public static final double INTAKE_SPEED_PERCENT = 0.8;
    public static final double PLACE_SPEED_PERCENT = 0.6;

    private GenericEntry disableVoltageLimiting;

    /**
     * Initialize the drivetrain subsystem
     * 
     * @param moduleInfo info for the swerve modules, in the order fl, fr, bl, br.
     * @param driveGearRatio The number of times the actual wheel rotates for each rotation of the drive motor.
     */
    public void init(SwerveModuleSetupInfo[] moduleInfo, double driveGearRatio) {
        initializeSwerveModules(moduleInfo, driveGearRatio);

        // Pass in the coordinates of each wheel relative to the center of the bot.
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FL
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FR
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // BR
        );

        OzoneImu pigeon = SubsystemManager.getInstance().getImu();
        
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getRotation2d(), getModulePositions(), new Pose2d(0,0,new Rotation2d()));
        // poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, Units.degreesToRadians(5)), 
        //     new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.01)),
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, Units.degreesToRadians(30))
        // );

        anglePid.enableContinuousInput(0, 360);
        anglePid.setTolerance(3);

        setDefaultCommand(new DriveCommand(this));

        // Add the encoder readings to shuffleboard
        tab.addNumber("FL angle", () -> frontLeftModule.getAngle().getDegrees());
        tab.addNumber("FR angle", () -> frontRightModule.getAngle().getDegrees());
        tab.addNumber("BL angle", () -> backLeftModule.getAngle().getDegrees());
        tab.addNumber("BR angle", () -> backRightModule.getAngle().getDegrees());
        tab.addBoolean("Is Moving", this::isMoving);
        tab.add(field);

        tab.addNumber("FL Speed", frontLeftModule::getVelocity);
        tab.addNumber("FR Speed", frontRightModule::getVelocity);
        tab.addNumber("BL Speed", backLeftModule::getVelocity);
        tab.addNumber("BR Speed", backRightModule::getVelocity);
        tab.addNumber("FL Target Angle", frontLeftModule::getTargetAngle);
        tab.addNumber("FR Target Angle", frontRightModule::getTargetAngle);
        tab.addNumber("BL Target Angle", backLeftModule::getTargetAngle);
        tab.addNumber("BR Target Angle", backRightModule::getTargetAngle);

        tab.addNumber("FL Position", () -> frontLeftModule.getPosition().distanceMeters);
        tab.addNumber("FR Position", () -> frontRightModule.getPosition().distanceMeters);
        tab.addNumber("BL Position", () -> backLeftModule.getPosition().distanceMeters);
        tab.addNumber("BR Position", () -> backRightModule.getPosition().distanceMeters);

        disableVoltageLimiting = tab.add("Disable VLS", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    /**
     * Do not call this from outside of drivetrain!
     * <p>
     * Initialize the swerve modules
     * 
     * @param moduleInfo info for the swerve modules, in the order fl, fr, bl, br.
     * @param driveGearRatio The number of times the actual wheel rotates for each rotation of the drive motor.
     * @throws Exception If there is an issue acquiring a port.
     */
    protected abstract void initializeSwerveModules(SwerveModuleSetupInfo[] moduleInfo, double driveGearRatio);

    @Override
    public void periodic() {

        double voltageLimitedSpeed = 1;
        double currentVoltage = SubsystemManager.getInstance().getPdp().getVoltage();
        // if(!disableVoltageLimiting.getBoolean(true) && currentVoltage <= 8.5) {
        //     voltageLimitedSpeed = Math.max(0, Math.pow((currentVoltage - 8.5), 1.6) * -40 + 100);
        // } else {
        //     voltageLimitedSpeed = 1;
        // }
        // speedPercent = Math.min(desiredSpeedPercent, voltageLimitedSpeed);

        SmartDashboard.putNumber("VLS", voltageLimitedSpeed);
        SmartDashboard.putNumber("speed percent", speedPercent);
        SmartDashboard.putNumber("desired speed percent", speedPercent);
        // Run the drive command periodically
        // field.setRobotPose(
        //     poseEstimator.getEstimatedPosition().getX(),
        //     poseEstimator.getEstimatedPosition().getY(),
        //     poseEstimator.getEstimatedPosition().getRotation()
        // );
        // field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    /** 
     * Drive the robot with percent output given a ChassisSpeeds object
     * <p>
     * This should be called periodically even if the input is not changing.
     * 
     * @param speeds Chassis speeds with vx and vy <= max linear speed and omega < max rotation speed
    */
    public void drive(ChassisSpeeds speeds, boolean fieldOriented) {
        if(isInBrakeMode) {
            // Rotate all the wheels to point to the center of the bot so we are hard to move.
            frontLeftModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
            frontRightModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(145)));
            backLeftModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
            backRightModule.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
            return;
        }
        OzoneImu pigeon = SubsystemManager.getInstance().getImu();
        if(fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond, 
                Rotation2d.fromDegrees(pigeon.getAngle())
            );
        }

        SmartDashboard.putNumber("Target angle: ", targetAngle);
        if(!Double.isNaN(targetAngle)) {
            speeds.omegaRadiansPerSecond = anglePid.calculate(pigeon.getAngle());
            isAtTargetAngle = anglePid.atSetpoint();
        }
        // Negate the vyMetersPerSecond so that a positive value will drive in the positive y direction. This must be done
        // Because for the wheels, clockwise is positive.
        // TODO: Make counter-clockwise positive for swerve modules.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED); // Normalize wheel speeds so we don't go faster than 100%
        
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon.getRotation2d(), getModulePositions());
        
        // field.setRobotPose(
        //     poseEstimator.getEstimatedPosition().getX(),
        //     poseEstimator.getEstimatedPosition().getY(),
        //     pigeon.getRotation2d()
        // );
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        // Update SwerveModule states
        frontLeftModule.updateState(SwerveModuleState.optimize(states[0], frontLeftModule.getAngle()));
        frontRightModule.updateState(SwerveModuleState.optimize(states[1], frontRightModule.getAngle()));
        backLeftModule.updateState(SwerveModuleState.optimize(states[2], backLeftModule.getAngle()));
        backRightModule.updateState(SwerveModuleState.optimize(states[3], backRightModule.getAngle()));
    }

    /**
     * Determine if the robot is stopped, or moving very slowly.
     * <p>
     * This is mainly for ignoring vision measurements while we are moving.
     * 
     * @return
     */
    public boolean isMoving() {
        boolean isMoving = false;
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        if(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)) > IS_MOVING_TRANSLATION_TOLERANCE) {
            isMoving = true;
        } else if(Math.abs(speeds.omegaRadiansPerSecond) > IS_MOVING_ROTATION_TOLERANCE) {
            isMoving = true;
        }
        return isMoving;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    public void stop() {
        drive(new ChassisSpeeds(), false);
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /**
     * Set the target angle for the bot to rotate to.
     * 
     * @param targetAngle The target angle.
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle.getDegrees();
        anglePid.setSetpoint(this.targetAngle);
    }

    /**
     * Remove the target angle so that the bot can freely rotate.
     */
    public void removeTargetAngle() {
        targetAngle = Double.NaN;
        anglePid.reset();
        isAtTargetAngle = false;
    }

    public boolean hasTargetAngle() {
        return !Double.isNaN(targetAngle);
    }

    public boolean atTargetAngle() {
        return isAtTargetAngle;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

    public void setSpeedPercent(double speedPercent) {
        desiredSpeedPercent = MathUtil.clamp(speedPercent, -1, 1);
    }

    /**
     * Turn field oriented on or off
     * 
     * @param val true to turn field oriented on, false to turn it off.
     */
    public void setFieldOriented(boolean val) {
        fieldOrientedToggle.setBoolean(val);
    }

    /**
     * Determine if the bot is in field oriented drive mode
     * 
     * @return Returns true if in field oriented, otherwise, false.
     */
    public boolean getFieldOriented() {
        return fieldOrientedToggle.getBoolean(true);
    }

    /**
     * Get the current estimated position of the robot in meters.
     * <p>
     * x+ is forwards, y+ is right.
     * 
     * @return The estimated position of the bot.
     */
    public Pose2d getLocation() {
        // return new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), SubsystemManager.getInstance().getImu().getRotation2d());
        return poseEstimator.getEstimatedPosition();
    }

    public void resetLocation(Pose2d botLocation) {
        poseEstimator.resetPosition(botLocation.getRotation(), getModulePositions(), botLocation);
        SubsystemManager.getInstance().getImu().setReset(botLocation.getRotation());
    }

    /**
     * @return Returns PoseEstimator
     */


    public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
        return poseEstimator;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void enableBrakeMode() {
        isInBrakeMode = true;
    }

    public void disableBrakeMode() {
        isInBrakeMode = false;
    }

    public boolean isInBrakeMode() {
        return isInBrakeMode;
    }

    public void setIsFollowingPath(boolean isFollowingPath) {
        this.isFollowingPath = isFollowingPath;
    }

    public boolean getIsFollowingPath() {
        return isFollowingPath;
    }
}
