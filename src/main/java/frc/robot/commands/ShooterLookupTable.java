package frc.robot.commands;

import java.util.Map;
// import java.util.Comparator;
// import java.util.List;
// import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;

public class ShooterLookupTable extends SequentialCommandGroup {
    private Kinesthetics s_Kinesthetics;
    private Shooter s_Shooter;
    private Swerve s_Swerve;
    private double desiredPitch;

    public ShooterLookupTable(Kinesthetics k, Shooter s, Swerve sw) {
        s_Kinesthetics = k;
        s_Shooter = s;
        s_Swerve = sw;
        desiredPitch = bestPitch(k);
        setupCommands();
    }

    /* Sets up the commands for the shooter lookup table */
    private void setupCommands() {
        addRequirements(s_Kinesthetics, s_Shooter, s_Swerve);
        addCommands(
            new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(desiredPitch, Constants.Shooter.leftSpeed, Constants.Shooter.rightSpeed), true),
                s_Swerve.new ChangeYaw(this::calculateYawX, this::calculateYawY, this::getRequiredYaw),
                new WaitUntilCommand(() -> s_Shooter.atState(desiredPitch, Constants.Shooter.leftSpeed, Constants.Shooter.rightSpeed))),
            s_Shooter.new ChangeNeck(SpinState.FW));
    }

    /** @return The x coordinate for the yaw calculation */
    private double calculateYawX() {
        return s_Kinesthetics.getPose().getX();
    }

    /** @return The y coordinate for the yaw calculation */
    private double calculateYawY() {
        return s_Kinesthetics.getPose().getY();
    }

    /** @return The required yaw angle in radians */
    private double getRequiredYaw() {
        Transform3d difference = getDifference(s_Kinesthetics);
        return Math.atan2(difference.getY(), difference.getX());
    }

    /**
     * Calculates the 3D spacial difference between the robot's current pose and the target pose
     * @return The difference as a Transform3d object
     */
    private static Transform3d getDifference(Kinesthetics k) {
        Pose3d currentPose = new Pose3d(k.getPose());
        Pose3d targetPose = Constants.Environment.speakers.get(k.getAlliance());
        return currentPose.minus(targetPose);
    }

    /**
     * Calculates the distance from the robot to the target
     * @return The distance in meters
     */
    private static double getDistance(Kinesthetics k) {
        Transform3d difference = getDifference(k);
        double distance = Math.hypot(difference.getX(), difference.getY());
        SmartDashboard.putNumber("Distance from speaker", distance);
        return distance;
    }

    /**
     * Determines the best pitch for the shooter based on the distance to the target
     * @return The calculated shooter pitch
     */
    private static double bestPitch(Kinesthetics k) {
        double currentDistance = getDistance(k);

        double closestMatchDistance = 0;
        double secClosestMatchDistance = 0;
        double closestMatchPitch = 0;
        double secClosestMatchPitch = 0;

        double closestDistanceDiff = Double.MAX_VALUE;
        double secClosestDistanceDiff = Double.MAX_VALUE;

        for (Map.Entry<Double, Double> entry : RobotContainer.shooterTable.entrySet()) {
            double key = entry.getKey();
            double value = entry.getValue();
            double distanceDiff = Math.abs(key - currentDistance);

            if (distanceDiff < closestDistanceDiff) {
                secClosestMatchDistance = closestMatchDistance;
                secClosestMatchPitch = closestMatchPitch;
                secClosestDistanceDiff = closestDistanceDiff;

                closestMatchDistance = key;
                closestMatchPitch = value;
                closestDistanceDiff = distanceDiff;
            } else if (distanceDiff < secClosestDistanceDiff) {
                secClosestMatchDistance = key;
                secClosestMatchPitch = value;
                secClosestDistanceDiff = distanceDiff;
            }
        }

        if (secClosestMatchDistance == 0 && secClosestMatchPitch == 0) {
            return closestMatchPitch;
        }

        double interpolationFactor = closestDistanceDiff / (closestDistanceDiff + secClosestDistanceDiff);
        return MathUtil.interpolate(closestMatchPitch, secClosestMatchPitch, interpolationFactor);
    }

    /* Alternate approach to the above method */
    /*
    private static double bestPitch2(Kinesthetics k) {
        double currentDistance = getDistance(k);
        List<Map.Entry<Double, Double>> sortedEntries = RobotContainer.shooterTable.entrySet().stream()
                .sorted(Comparator.comparingDouble(entry -> Math.abs(entry.getKey() - currentDistance)))
                .collect(Collectors.toList());

        double closestMatchPitch = sortedEntries.get(0).getValue();
        
        if (sortedEntries.size() == 1 || sortedEntries.get(1).getKey().equals(sortedEntries.get(0).getKey())) {
            return closestMatchPitch;
        }

        double closestDistanceDiff = Math.abs(sortedEntries.get(0).getKey() - currentDistance);
        double secClosestDistanceDiff = Math.abs(sortedEntries.get(1).getKey() - currentDistance);
        double secClosestMatchPitch = sortedEntries.get(1).getValue();
        
        double interpolationFactor = closestDistanceDiff / (closestDistanceDiff + secClosestDistanceDiff);
        return MathUtil.interpolate(closestMatchPitch, secClosestMatchPitch, interpolationFactor);
    }
    */
}