package frc.robot.subsystems.drivetrain;

/** Add your docs here. */
public class SwerveModuleSetupInfo {
    private int driveMotorCanId;
    private int angleMotorCanId;
    private int encoderPort;

    private double wheelOffset;

    public SwerveModuleSetupInfo(int driveMotorCanId, int angleMotorCanId, int encoderPort, double wheelOffset) {
        this.driveMotorCanId = driveMotorCanId;
        this.angleMotorCanId = angleMotorCanId;
        this.encoderPort = encoderPort;
        this.wheelOffset = wheelOffset;
    }

    public int getDriveMotorCanId() {
        return driveMotorCanId;
    }

    public int getAngleMotorCanId() {
        return angleMotorCanId;
    }

    public int getEncoderPort() {
        return encoderPort;
    }

    public double getWheelOffset() {
        return wheelOffset;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + driveMotorCanId;
        result = prime * result + angleMotorCanId;
        result = prime * result + encoderPort;
        long temp;
        temp = Double.doubleToLongBits(wheelOffset);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (getClass() != obj.getClass()) {
            return false;
        }

        SwerveModuleSetupInfo other = (SwerveModuleSetupInfo) obj;

        if (driveMotorCanId != other.driveMotorCanId) {
            return false;
        } else if (angleMotorCanId != other.angleMotorCanId) {
            return false;
        } else if (encoderPort != other.encoderPort) {
            return false;
        } else if (Double.doubleToLongBits(wheelOffset) != Double.doubleToLongBits(other.wheelOffset)) {
            return false;
        }
        
        return true;
    }

}
