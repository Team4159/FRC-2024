package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SpkAlign extends SequentialCommandGroup {

    /**
     * Constructs the SpkAlign command to orient the robot towards the alliance speaker
     *
     * @param kinesthetics The kinesthetics subsystem to obtain the robot's current pose
     * @param swerve       The swerve drive subsystem to execute the yaw change
     */
    public SpkAlign(Kinesthetics kinesthetics, Swerve swerve) {
        // Require the kinesthetics and swerve subsystems
        addRequirements(kinesthetics, swerve);

        // Change yaw towards the target
        addCommands(swerve.new ChangeYaw(
            () -> kinesthetics.getPose().getX(),
            () -> kinesthetics.getPose().getY(),
            () -> calculateRequiredYaw(kinesthetics)));
    }

    /**
     * Calculates the required yaw to face the alliance speaker
     *
     * @param kinesthetics The kinesthetics subsystem to get the current pose
     * @return The required yaw in radians
     */
    private double calculateRequiredYaw(Kinesthetics kinesthetics) {
        Transform3d difference = calculatePoseDifference(kinesthetics);
        return Math.atan2(difference.getY(), difference.getX());
    }

    /**
     * Calculates the difference between the robot's current pose and the target speaker pose
     *
     * @param kinesthetics The kinesthetics subsystem to get the current pose
     * @return The Transform3d representing the difference to the target
     */
    private static Transform3d calculatePoseDifference(Kinesthetics kinesthetics) {
        Pose3d currentPose = new Pose3d(kinesthetics.getPose());
        Pose3d speakerPose = Constants.Environment.speakers.get(kinesthetics.getAlliance());
        return currentPose.minus(speakerPose);
    }
}