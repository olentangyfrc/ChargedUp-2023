package frc.robot.auton;

import java.util.Arrays;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class AutonPaths {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION);

    private PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("test path", CONSTRAINTS);


    private CommandBase testTrajectoryCommand;

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
    public CommandBase wrapPathCommand(Command command) {
        return (new InstantCommand(() -> drivetrain.setIsFollowingPath(true))).andThen(command).andThen(() -> drivetrain.setIsFollowingPath(false));
    }

    public CommandBase getTestTrajectoryCommand() {
        return testTrajectoryCommand;
    }

    /**
     * Get a command to follow a path to a position, assuming the drivetrain is not moving at the start.
     * 
     * @param position The position to drive to
     * @return A command to drive to the given position.
     */
    public CommandBase pathToPositionCommand(Pose2d position)  {
        return pathToPositionCommand(position, new ChassisSpeeds());
    }

    /**
     * Get a command to follow a path to a position
     * 
     * @param position The position to drive to
     * @param currentSpeeds The current speeds of the drivetrain.
     * @return A command to drive to the given position.
     */
    public CommandBase pathToPositionCommand(Pose2d position, ChassisSpeeds currentSpeeds) {
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

        Translation2d offset = position.getTranslation().minus(drivetrain.getLocation().getTranslation());

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            CONSTRAINTS,
            new PathPoint(drivetrain.getLocation().getTranslation(), offset.getAngle(), drivetrain.getLocation().getRotation()),
            new PathPoint(position.getTranslation(), offset.getAngle(), position.getRotation())
        );
        System.out.println(offset.getAngle());
        displayPath(trajectory);
        return wrapPathCommand(builder.followPath(trajectory));
    }

    public static void displayPath(PathPlannerTrajectory trajectory) {
        int fieldWidth = 5, fieldHeight = 5;
        int resolution = 10; // Pixels per meter
        char[][] grid = new char[fieldHeight * resolution][fieldWidth * resolution];
        for(int y = 0; y < grid.length; y++) {
            Arrays.fill(grid[y], '-');
        }

        for(State state : trajectory.getStates()) {
            int y = (int) MathUtil.clamp(Math.round(state.poseMeters.getY() * resolution), 0, fieldHeight * resolution - 1);
            int x = (int) MathUtil.clamp(Math.round(state.poseMeters.getX() * resolution), 0, fieldWidth * resolution - 1);
            if(grid[y][x] == '-' || state.equals(trajectory.getEndState())) {
                grid[y][x] = state.equals(trajectory.getInitialState())? 'S' : state.equals(trajectory.getEndState())? 'E' : 'O';
            }
        }


        for(int y = grid.length - 1; y >= 0; y--) {
            String line = "";
            for(int x = 0; x < grid[y].length; x++) {
                line += grid[y][x];
            }
            System.out.println(line);
        }
    }
}