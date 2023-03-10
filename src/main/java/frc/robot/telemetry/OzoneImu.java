package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is to be used for different Pigeon IMUs and NavXs
 */
public abstract class OzoneImu {
    protected boolean isInverted = false;

    public OzoneImu() {
        Shuffleboard.getTab("Gyro").addNumber("Yaw", this::getAngle);
    }

    public abstract double getAngle();
    public abstract double getPitch();
    public abstract double getRoll();
    public abstract void reset();

    public abstract void resetPitch();

    public abstract void resetRoll();

    public abstract void setReset(Rotation2d angle);

    /**
     * @return Get the Yaw angle as a rotation2d (ccw is positive)
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Set whether or not the yaw angle is inverted
     * 
     * @param inverted true if the yaw should be inverted
     */
    public void setIsInverted(boolean inverted) {
        this.isInverted = inverted;
    }

    /**
     * Determine whether or not the yaw angle is inverted
     * 
     * @return True if the yaw angle is inverted
     */
    public boolean getIsInverted() {
        return isInverted;
    }
}
