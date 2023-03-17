package frc.robot.telemetry;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Pigeon2 extends OzoneImu {
    private WPI_Pigeon2 imu;

    private double pitchOffset = 0;
    private double rollOffset = 0;
    
    public Pigeon2(int deviceID) {
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
        // double offset = -3;
        // if (imu.getPitch() >= offset && imu.getPitch() < 0) {
        //     return 0;
        // }
        return imu.getRoll() - pitchOffset;
    }

    @Override
    public double getRoll() {
        return imu.getPitch() - rollOffset;
    }

    @Override
    public void setReset(Rotation2d angle) {
        // Resets the gyro angle to a given value
        imu.setYaw(angle.getDegrees());
    }

    @Override
    public void reset() {
        if(DriverStation.getAlliance() == Alliance.Blue) {
            setReset(new Rotation2d());
        } else {
            setReset(Rotation2d.fromDegrees(180));
        }
    }

    @Override
    public void resetPitch() {
        rollOffset = getRoll();
    }

    @Override
    public void resetRoll() {
        pitchOffset = getPitch();
    }
}