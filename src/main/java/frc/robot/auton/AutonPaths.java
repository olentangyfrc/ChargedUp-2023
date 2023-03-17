package frc.robot.auton;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.DeployIntake;
import frc.robot.subsystems.activeintake.commands.RetractIntake;
import frc.robot.subsystems.activeintake.commands.StartIntake;
import frc.robot.subsystems.activeintake.commands.StopIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;

// TODO: Separate Auto routines into another class
public class AutonPaths {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION);

    private SwerveDrivetrain drivetrain;

    private Map<AutoTrajectory, PathPlannerTrajectory> trajectoryMap;

    private SwerveAutoBuilder builder;

    private Map<String, Command> eventMap;

    public AutonPaths(SwerveDrivetrain drivetrain, ActiveIntake intake, Claw claw, ClawPitch clawPitch, Elevator elevator) {
        this.drivetrain = drivetrain;

        eventMap = Map.of(
            "DeployIntake", Commands.parallel(new DeployIntake(intake), new StartIntake(intake)),
            "RetractIntake", Commands.parallel(new RetractIntake(intake), new StopIntake(intake))
        );

        trajectoryMap = new HashMap<AutoTrajectory, PathPlannerTrajectory>();

        builder = new SwerveAutoBuilder(
            drivetrain::getLocation,
            drivetrain::resetLocation,
            drivetrain.translationPidConstants,
            drivetrain.rotationPidConstants,
            (speeds) -> drivetrain.drive(speeds, false),
            eventMap,
            true,
            drivetrain
        );

        
        generatePaths();
    }

    private void generatePaths() {
        for(AutoTrajectory trajectory : AutoTrajectory.values()) {
            trajectoryMap.put(trajectory, PathPlanner.loadPath(trajectory.name(), CONSTRAINTS));
        }
    }

    public CommandBase followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return (new InstantCommand(() -> drivetrain.resetLocation(trajectory.getInitialPose()))).andThen(wrapPathCommand(builder.followPathWithEvents(trajectory)));
    }

    /**
     * Update a path following command to tell the drivetrain when it is path following.
     * 
     * @param command The command to wrap
     * @return The wrapped command.
     */
    public CommandBase wrapPathCommand(Command command) {
        return (new InstantCommand(() -> {
            drivetrain.setIsFollowingPath(true);
        })).andThen(command).andThen(() -> drivetrain.setIsFollowingPath(false));
    }

    public PathPlannerTrajectory getTrajectory(AutoTrajectory trajectory) {
        return trajectoryMap.getOrDefault(trajectory, null);
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

        // Translation2d offset = position.getTranslation().minus(drivetrain.getLocation().getTranslation());
        SwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();

        ChassisSpeeds speeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getModuleStates());
        
        Rotation2d driveAngle = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            CONSTRAINTS,
            new PathPoint(drivetrain.getLocation().getTranslation(), driveAngle.rotateBy(Rotation2d.fromDegrees(180)), drivetrain.getLocation().getRotation(), speed),
            new PathPoint(position.getTranslation(), Rotation2d.fromDegrees(180), position.getRotation())
        );
        return wrapPathCommand(builder.followPath(trajectory));
    }

    

    public static enum AutoTrajectory {
        GetGamepieceOne,
        GetGamepieceTwo,
        GetGamepieceThree,
        GetGamepieceFour,
        
        TopToChargingStation,
        MiddleToChargingStation,
        BottomToChargingStation,

        TopTaxi,
        BottomTaxi,
        OnChargingStation
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