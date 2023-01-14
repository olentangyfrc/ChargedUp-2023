// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.drivetrain.modules.CANSparkMaxModule;

public class SparkMaxDrivetrain extends SwerveDrivetrain {

    @Override
    public void initializeSwerveModules(Map<String, Integer> portAssignments, Map<String, Double> wheelOffsets) throws Exception {

        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);
        thetaController = new PIDController(1, 0, 0);

        // Initialize swerve modules
        frontLeftModule = new CANSparkMaxModule(
            portAssignments.get("FL.SwerveMotor"),
            portAssignments.get("FL.DriveMotor"),
            portAssignments.get("FL.Encoder"),
            wheelOffsets.get("FL"),
            MAX_LINEAR_SPEED
        );

        frontRightModule = new CANSparkMaxModule(
            portAssignments.get("FR.SwerveMotor"),
            portAssignments.get("FR.DriveMotor"),
            portAssignments.get("FR.Encoder"),
            wheelOffsets.get("FR"),
            MAX_LINEAR_SPEED
        );

        backLeftModule = new CANSparkMaxModule(
            portAssignments.get("BL.SwerveMotor"),
            portAssignments.get("BL.DriveMotor"),
            portAssignments.get("BL.Encoder"),
            wheelOffsets.get("BL"),
            MAX_LINEAR_SPEED
        );

        backRightModule = new CANSparkMaxModule(
            portAssignments.get("BR.SwerveMotor"),
            portAssignments.get("BR.DriveMotor"),
            portAssignments.get("BR.Encoder"),
            wheelOffsets.get("BR"),
            MAX_LINEAR_SPEED
        );

    }
}
