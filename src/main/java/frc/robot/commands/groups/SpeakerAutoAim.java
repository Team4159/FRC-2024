package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerAutoAim extends Command {
    private Kinesthetics kinesthetics;
    private Swerve s_Swerve;
    private double desiredYaw;
    private Shooter s_Shooter;
    private double desiredPitch;
    private double desiredSpin; // radians/second

    public SpeakerAutoAim(Kinesthetics k, Swerve sw, Shooter sh) { // TODO: needs to accept translation args to continue moving
        kinesthetics = k;
        s_Swerve = sw;
        s_Shooter = sh;
        addRequirements(kinesthetics, s_Swerve, s_Shooter);
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Field.speakers.get(k.getAlliance()));
    }

    @Override
    public void execute() {
        Transform3d transform = getDifference(kinesthetics);
        desiredPitch = transform.getRotation().getY() + 0; // TODO: Calculate ascension for shooting arc
        desiredYaw = transform.getRotation().getZ() + 0; // TODO: Calculate angle offset to account for velocity and anglular velocity
        desiredSpin = 10; // TODO: Calculate firing velocity
        kinesthetics.lockRotation(true);
        s_Swerve.drive(new Translation2d(), desiredYaw, true, false);
        s_Shooter.setGoalPitch(desiredPitch);
        s_Shooter.setGoalSpin(desiredSpin);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(kinesthetics.getHeading().getRadians() - desiredYaw) < Constants.Swerve.yawTolerance
            && Math.abs(s_Shooter.getPitch() - desiredPitch) < Constants.Shooter.pitchTolerance
            && Math.abs(s_Shooter.getSpin() - desiredSpin) < Constants.Shooter.spinTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        kinesthetics.lockRotation(false);
        if (interrupted) {
            s_Shooter.setGoalPitch(Constants.Shooter.restingPitch);
            s_Shooter.stopSpin();
        }
        super.end(interrupted);
    }

    public static boolean isInRange(Kinesthetics k) {
        Translation2d offset = getDifference(k).getTranslation().toTranslation2d();
        return offset.getY() < 0.1 && offset.getX() < 3; // TODO: plot out valid range
    }
}
