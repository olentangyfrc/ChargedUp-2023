// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;

import frc.robot.subsystems.drivetrain.modules.SingleFalconModule;

/** Add your docs here. */
public class SingleFalconDrivetrain extends SwerveDrivetrain {

    @Override
    public void initializeSwerveModules(SwerveModuleSetupInfo[] moduleInfo, double driveGearRatio) {

        // translationPidConstants = new PIDConstants(1.5, 0, 0);
        // rotationPidConstants = new PIDConstants(0.3, 0, 0);
        translationPidConstants = new PIDConstants(0.2, 0, 0);
        rotationPidConstants = new PIDConstants(2, 0, 0);
                
        // Initialize swerve modules
        frontLeftModule = new SingleFalconModule(
            moduleInfo[0].getAngleMotorCanId(),
            moduleInfo[0].getDriveMotorCanId(),
            moduleInfo[0].getEncoderPort(),
            moduleInfo[0].getWheelOffset(),
            driveGearRatio,
            MAX_LINEAR_SPEED
        );

        frontRightModule = new SingleFalconModule(
            moduleInfo[1].getAngleMotorCanId(),
            moduleInfo[1].getDriveMotorCanId(),
            moduleInfo[1].getEncoderPort(),
            moduleInfo[1].getWheelOffset(),
            driveGearRatio,
            MAX_LINEAR_SPEED
        );

        backLeftModule = new SingleFalconModule(
            moduleInfo[2].getAngleMotorCanId(),
            moduleInfo[2].getDriveMotorCanId(),
            moduleInfo[2].getEncoderPort(),
            moduleInfo[2].getWheelOffset(),
            driveGearRatio,
            MAX_LINEAR_SPEED
        );

        backRightModule = new SingleFalconModule(
            moduleInfo[3].getAngleMotorCanId(),
            moduleInfo[3].getDriveMotorCanId(),
            moduleInfo[3].getEncoderPort(),
            moduleInfo[3].getWheelOffset(),
            driveGearRatio,
            MAX_LINEAR_SPEED
        );

    }
}
