package frc.robot.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class AutonPaths {
    private PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("test path", new PathConstraints(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION));

    private Command testTrajectoryCommand;

    private SwerveDrivetrain drivetrain;

    public AutonPaths(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        generatePaths();
    }

    private void generatePaths() {
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            drivetrain::getLocation,
            drivetrain::resetLocation,
            drivetrain.translationPidConstants,
            drivetrain.rotationPidConstants,
            (speeds) -> drivetrain.drive(speeds, false),
            new HashMap<String, Command>(),
            true,
            drivetrain
        );
        
        testTrajectoryCommand = Commands.sequence(new InstantCommand(() -> drivetrain.resetLocation(testTrajectory.getInitialHolonomicPose())), wrapPathCommand(builder.followPath(testTrajectory)));
    }

    /**
     * Update a path following command to tell the drivetrain when it is path following.
     * 
     * @param command The command to wrap
     * @return The wrapped command.
     */
    public Command wrapPathCommand(Command command) {
        return (new InstantCommand(() -> drivetrain.setIsFollowingPath(true))).andThen(command).andThen(() -> drivetrain.setIsFollowingPath(false));
    }

    public Command getTestTrajectoryCommand() {
        return testTrajectoryCommand;
    }
}