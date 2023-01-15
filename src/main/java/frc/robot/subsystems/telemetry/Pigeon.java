package frc.robot.subsystems.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {
    private Pigeon2 imu;
    private boolean isInverted = false;
    
    public Pigeon(int deviceID){
        this.imu = new Pigeon2(deviceID);
    }

    public double getAngle() {
        if(isInverted){
            return -imu.getYaw();
        } else {
            return imu.getYaw();
        }
    }

    public double getPitch() {
        return imu.getPitch();
    }

    public double getRoll() {
        return imu.getRoll();
    }

    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void reset(){
        imu.setYaw(0);
    }

    public void setReset(Rotation2d angle){
        // Resets the gyro angle to a given value
        imu.setYaw(angle.getDegrees());
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted; 
    }

    public boolean getInverted(){
        return isInverted;
    }
}
