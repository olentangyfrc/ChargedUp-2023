package frc.robot.subsystems.telemetry;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2 extends OzoneImu{
    private WPI_Pigeon2 imu;
    
    public Pigeon2(int deviceID){
        this.imu = new WPI_Pigeon2(deviceID);
        imu.configFactoryDefault();
    }

    @Override
    public double getAngle() {
        if(isInverted){
            return -imu.getYaw();
        } else {
            return imu.getYaw();
        }
    }

    @Override
    public double getPitch() {
        return imu.getPitch();
    }

    @Override
    public double getRoll() {
        return imu.getRoll();
    }

    @Override
    public void setReset(Rotation2d angle){
        // Resets the gyro angle to a given value
        imu.setYaw(angle.getDegrees());
    }
}
