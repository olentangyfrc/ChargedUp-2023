package frc.robot.telemetry;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends OzoneImu {
    private WPI_PigeonIMU pigeon;

    public Pigeon(int deviceID) {
        pigeon = new WPI_PigeonIMU(deviceID);
    }

    @Override
    public double getAngle() {
        return pigeon.getYaw() % 360;
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch() % 360;
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll() % 360;
    }

    @Override
    public void setReset(Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }

    /**
     * Enter gyro calibration mode.
     * <p>
     * Wait at least 4 seconds after calling this to move the bot.
     */
    public void calibrate() {
        new Thread(() -> {
            while (!pigeon.getState().equals(PigeonState.Ready));
            pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
        }).start();
    }

    /**
     * Enter accelerometer calibration mode.
     * <p>
     * Make sure that the gyro is completely level and still for 10s after this is called.
     */
    public void calibrateAccelerometer() {
        new Thread(() -> {
            while (!pigeon.getState().equals(PigeonState.Ready));
            pigeon.enterCalibrationMode(CalibrationMode.Accelerometer);
        }).start();
    }
    
}
