// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class AutoRoutineManager {
    private Map<AutoRoutine, CommandBase> routineMap;
    private SendableChooser<AutoRoutine> autoChooser = new SendableChooser<AutoRoutine>();

    public AutoRoutineManager() {
        routineMap = new HashMap<AutoRoutine, CommandBase>();

        // add paths to chooser here
        for(AutoRoutine routine : AutoRoutine.values()) {
            autoChooser.addOption(routine.name(), routine);
        }

        Shuffleboard.getTab("Auton").add("AutoChooser", autoChooser);
    }
    
    public CommandBase getRoutine(AutoRoutine routine) {
        return routineMap.getOrDefault(routine, new InstantCommand());
    }

    public CommandBase getSelectedRoutine() {
        return getRoutine(autoChooser.getSelected());
    }

    public static enum AutoRoutine {
    }

}
