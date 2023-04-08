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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.commands.MoveElevator;

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
            "DeployIntake", Commands.sequence(
                new DeployIntake(intake),
                new StartIntake(intake),
                new ParallelCommandGroup(
                  new MoveElevator(elevator, elevator.getPositionValue(ElevatorPosition.LOW) + 0.1),
                  new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    Commands.runOnce((intake::setOff))
                  )
                )
              ),
            "RetractIntake", Commands.parallel(new RetractIntake(intake), new StopIntake(intake))
        );

        trajectoryMap = new HashMap<AutoTrajectory, PathPlannerTrajectory>();

        builder = new SwerveAutoBuilder(
            drivetrain::getLocation,
            drivetrain::resetLocation,
            drivetrain.translationPidConstants,
            // new PIDConstants(0, 0, 0),
            // new PIDConstants(0, 0, 0),
            drivetrain.rotationPidConstants,
            (speeds) -> drivetrain.drive(speeds, false),
            eventMap,
            false,
            drivetrain
        );

        generatePaths();
    }

    private void generatePaths() {
        for(AutoTrajectory trajectory : AutoTrajectory.values()) {
            trajectoryMap.put(trajectory, PathPlanner.loadPathGroup(trajectory.name(), CONSTRAINTS).get(0));
        }
    }

    public CommandBase followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return (new InstantCommand(() -> drivetrain.resetLocation(trajectory.getInitialHolonomicPose()))).andThen(wrapPathCommand(builder.followPathWithEvents(trajectory)));
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
        System.out.println(((DriverStation.getAlliance() == Alliance.Red)? "Red" : "") + trajectory.name() + "______)ASty7jw497y9w");
        return trajectoryMap.getOrDefault(AutoTrajectory.valueOf(((DriverStation.getAlliance() == Alliance.Red)? "Red" : "") + trajectory.name()), null);

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
            false,
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
        GetGamepieceFour,

        RedGetGamepieceOne,
        RedGetGamepieceFour,

        GetGamepieceOneAndBalance,

        RedGetGamepieceOneAndBalance,
        
        TopTaxi,
        BottomTaxi,
        OnChargingStation,
        OverAndBack,

        RedTopTaxi,
        RedBottomTaxi,
        RedOnChargingStation,
        RedOverAndBack
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