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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.auton.AutonPaths.AutoTrajectory;
import frc.robot.auton.routines.PlaceConeAndGoBack;
import frc.robot.auton.routines.bottom.BottomPlace;
import frc.robot.auton.routines.bottom.BottomPlaceAndTaxi;
import frc.robot.auton.routines.bottom.BottomTwoPiece;
import frc.robot.auton.routines.middle.MiddleOverAndBack;
import frc.robot.auton.routines.middle.MiddlePlace;
import frc.robot.auton.routines.middle.MiddlePlaceAndEngage;
import frc.robot.auton.routines.top.TopPlace;
import frc.robot.auton.routines.top.TopPlaceAndBalance;
import frc.robot.auton.routines.top.TopPlaceAndTaxi;
import frc.robot.auton.routines.top.TopTwoPiece;
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
        paths = new AutonPaths(drivetrain, intake, claw, clawPitch, elevator);
        routineMap = new HashMap<AutoRoutine, CommandBase>();
        
        routineMap.put(AutoRoutine.NOTHING, new InstantCommand());
        routineMap.put(AutoRoutine.TopPlaceAndTaxi, new TopPlaceAndTaxi(intake, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.TopPlace, new TopPlace(drivetrain, intake, claw, clawPitch, elevator));
        routineMap.put(AutoRoutine.TopTwoPiece, new TopTwoPiece(intake, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.BottomPlace, new BottomPlace(drivetrain, intake, claw, clawPitch, elevator));
        routineMap.put(AutoRoutine.BottomPlaceAndTaxi, new BottomPlaceAndTaxi(intake, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.BottomTwoPiece, new BottomTwoPiece(intake, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.MiddlePlace, new MiddlePlace(drivetrain, intake, claw, clawPitch, elevator));
        routineMap.put(AutoRoutine.MiddlePlaceAndEngage, new MiddlePlaceAndEngage(intake, drivetrain, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.JustTopTaxi, new ProxyCommand(() -> paths.followTrajectoryCommand(paths.getTrajectory(AutoTrajectory.TopTaxi))));
        routineMap.put(AutoRoutine.PlaceConeAndGoBack, new PlaceConeAndGoBack(drivetrain, intake, claw, clawPitch, elevator));
        routineMap.put(AutoRoutine.MiddleOverAndBack, new MiddleOverAndBack(intake, drivetrain, claw, clawPitch, elevator, paths));
        routineMap.put(AutoRoutine.TopPlaceAndBalance, new TopPlaceAndBalance(intake, drivetrain, claw, clawPitch, elevator, paths));

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

        TopPlace,
        TopPlaceAndTaxi,
        TopTwoPiece,
        JustTopTaxi,
        TopPlaceAndBalance,

        BottomPlace,
        BottomPlaceAndTaxi,
        BottomTwoPiece,

        MiddlePlace,
        MiddlePlaceAndEngage,
        MiddleOverAndBack,

        PlaceConeAndGoBack
    }

}
