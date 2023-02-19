// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemManager;

/** Add your docs here. */
public class AutoNodeUtility {
    private static final Translation2d[] ROW_POSITIONS = {
        new Translation2d(1.9, 0.47),
        new Translation2d(1.9, 1.05),
        new Translation2d(1.9, 1.62),
        new Translation2d(1.9, 2.19),
        new Translation2d(1.9, 2.75),
        new Translation2d(1.9, 3.30),
        new Translation2d(1.9, 3.87),
        new Translation2d(1.9, 4.41),
        new Translation2d(1.9, 4.97),
    };

    public static Translation2d getRowPosition(int row) {
        if(row < 0 || row > 8) {
            return null;
        }
        return ROW_POSITIONS[8 - row];
    }

    public static int getRow(int node) {
        return (node % 9 == 0)? 8 : node % 9 - 1;
    }

    public static Pose2d getNodeDrivePosition(int node) {
        Translation2d translation = getRowPosition(getRow(node));
        Rotation2d rotation = Rotation2d.fromDegrees(0);

        if(node > 18) {
            rotation = Rotation2d.fromDegrees(180);
        }

        return new Pose2d(translation, rotation);
    }
}
