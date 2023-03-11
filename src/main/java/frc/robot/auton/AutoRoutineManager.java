// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.AutonPaths.AutoTrajectory;
import frc.robot.auton.routines.TopPlaceAndTaxi;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;

/** Add your docs here. */
public class AutoRoutineManager {
    private Map<AutoRoutine, CommandBase> routineMap;
    private SendableChooser<AutoRoutine> autoChooser = new SendableChooser<AutoRoutine>();
    private AutonPaths paths;

    public AutoRoutineManager(SwerveDrivetrain drivetrain, ActiveIntake intake, Claw claw, ClawPitch clawPitch, Elevator elevator) {
        paths = new AutonPaths(drivetrain);
        routineMap = Map.of(
            AutoRoutine.NOTHING, new InstantCommand(),
            AutoRoutine.TopPlaceAndTaxi, new TopPlaceAndTaxi(intake, claw, clawPitch, elevator, paths),
            AutoRoutine.OnChargingStation, paths.followTrajectoryCommand(paths.getTrajectory(AutoTrajectory.OnChargingStation))
        );

        // add paths to chooser here
        for(AutoRoutine routine : AutoRoutine.values()) {
            autoChooser.addOption(routine.name(), routine);
        }

        autoChooser.setDefaultOption("NOTHING", AutoRoutine.NOTHING);

        Shuffleboard.getTab("Auton").add("AutoChooser", autoChooser);
    }
    
    public CommandBase getRoutine(AutoRoutine routine) {
        return routineMap.getOrDefault(routine, new InstantCommand());
    }

    public CommandBase getSelectedRoutine() {
        return getRoutine(autoChooser.getSelected());
    }

    public static enum AutoRoutine {
        NOTHING,
        TopPlaceAndTaxi,
        OnChargingStation
    }

}
