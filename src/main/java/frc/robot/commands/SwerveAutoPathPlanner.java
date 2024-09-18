package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.*;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.RobotState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.AutoConfig;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Swerve;

public class SwerveAutoPathPlanner extends Command{
    private static final PPHolonomicDriveController controller = new PPHolonomicDriveController(
            new PIDConstants(AutoConfig.kPXController, 0, 0),
            new PIDConstants(AutoConfig.kPYController, 0, 0), 
            Constants.Swerve.AutoConfig.kMaxSpeedMetersPerSecond, 
            (Constants.Swerve.wheelBase)*Math.sqrt(2)/2);
    
    private Kinesthetics kinesthetics;
    private Swerve swerve;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private Supplier<Pose2d> endState;

    private final Timer timer = new Timer();

    public SwerveAutoPathPlanner(Kinesthetics k, Swerve s, Supplier<Pose2d> end) {
        kinesthetics = k;
        swerve = s;
        endState = end;
        List<Translation2d> bezier = PathPlannerPath.bezierFromPoses(new Pose2d(), new Pose2d(new Translation2d(), new Rotation2d(end.get().getRotation().getRadians())));
        path = new PathPlannerPath(
            bezier, 
            Constants.Swerve.AutoConfig.pathConstraints, 
            new GoalEndState(0, end.get().getRotation()));
        trajectory = new PathPlannerTrajectory(path, Constants.Swerve.swerveKinematics.toChassisSpeeds(s.getModuleStates()), kinesthetics.getPose().getRotation());
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
        super.initialize();
    }

    @Override
    public void execute() {
        if(endState != null && timer.get() % 1.0 == 0){
            // trajectory = TrajectoryGenerator.generateTrajectory(
            // kinesthetics.getPose(), 
            // new ArrayList<>(), 
            // endState.get(),
            // new TrajectoryConfig(AutoConfig.kMaxAccelerationMetersPerSecondSquared, AutoConfig.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Swerve.swerveKinematics));
        }
        //var state = trajectory.sample(timer.get());
        // swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        //     controller.calculate(kinesthetics.getPose(), state, state.poseMeters.getRotation())
        // ));
        swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            controller.calculateRobotRelativeSpeeds(new Pose2d(), trajectory.getEndState())));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted) swerve.stop();
        super.end(interrupted);
    }
}
