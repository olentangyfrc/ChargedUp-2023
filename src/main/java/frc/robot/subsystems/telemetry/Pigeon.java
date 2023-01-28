package frc.robot.subsystems.telemetry;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends OzoneImu {
    private WPI_PigeonIMU pigeon;
    private double pitchOffset = 0;
    private double rollOffset = 0;

    public Pigeon(int deviceID) {
        pigeon = new WPI_PigeonIMU(deviceID);
        pitchOffset = pigeon.getPitch();
        rollOffset = pigeon.getRoll();
    }

    @Override
    public double getAngle() {
        return pigeon.getYaw() % 360;
    }

    @Override
    public double getPitch() {
        return (pigeon.getPitch() % 360) - pitchOffset;
    }

    @Override
    public double getRoll() {
        return (pigeon.getRoll() % 360) - rollOffset;
    }

    @Override
    public void setReset(Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }

    @Override
    public void reset() {
        pigeon.setYaw(0);
        pitchOffset = pigeon.getPitch();
        rollOffset = pigeon.getRoll();
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
