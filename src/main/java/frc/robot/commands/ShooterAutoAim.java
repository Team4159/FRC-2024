package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ShooterAutoAim extends Command {
    private Kinesthetics kinesthetics;
    private Swerve s_Swerve;
    private Shooter s_Shooter;

    public ShooterAutoAim(Kinesthetics k, Swerve sw, Shooter sh) {
        kinesthetics = k;
        s_Swerve = sw;
        s_Shooter = sh;
    }

    private Transform3d getDifference() {
        return new Pose3d(kinesthetics.getPose()).minus(Constants.Field.speakers.get(kinesthetics.getAlliance()));
    }

    @Override
    public void execute() {
        Transform3d transform = getDifference();
        s_Swerve.drive(new Translation2d(), transform.getRotation().getZ() + 0, true, false); // TODO: Calculate angle offset to account for velocity and anglular velocity
        // Math.abs(getPitch() - goal) < Constants.Shooter.pitchTolerance
        s_Shooter.setGoalPitch(transform.getRotation().getY() + 0); // TODO: Calculate ascension for shooting arc
    }
}
