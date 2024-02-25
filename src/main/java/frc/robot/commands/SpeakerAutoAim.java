package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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
    private Shooter s_Shooter;

    private DoubleSupplier desiredTranslation;
    private DoubleSupplier desiredStrafe;
    private double desiredYaw;
    private double desiredPitch;
    private double desiredNoteVel; // radians/second

    public SpeakerAutoAim(Kinesthetics k, Swerve sw, Shooter sh, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        kinesthetics = k;
        s_Swerve = sw;
        s_Shooter = sh;
        desiredTranslation = translationSup;
        desiredStrafe = strafeSup;
        addRequirements(kinesthetics, s_Swerve, s_Shooter);
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(desiredTranslation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(desiredStrafe.getAsDouble(), Constants.stickDeadband);

        Transform3d transform = getDifference(kinesthetics);
        double roottwogh = Math.sqrt(2*Constants.Environment.G*transform.getZ()); // Z is vertical
        desiredPitch = Math.atan(roottwogh / (
            (transform.getY()*Constants.Environment.G)
            / roottwogh
            - kinesthetics.getVelocity().get(1, 0) // y velocity
        ));
        if (desiredPitch < 0) desiredPitch += Math.PI;
        desiredYaw = Math.atan(transform.getY()/transform.getX()); // TODO: Calculate angle offset to account for velocity and anglular velocity
        desiredNoteVel = Math.sqrt(
            2*Constants.Environment.G*transform.getZ()
            +
            Math.pow(
                (transform.getY()*Constants.Environment.G)
                / roottwogh
                - kinesthetics.getVelocity().get(1, 0) // y velocity
            , 2)
        );

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            desiredYaw, true, false
        );
        s_Shooter.setGoalPitch(desiredPitch);
        s_Shooter.setGoalSpin(desiredNoteVel);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(kinesthetics.getHeading().getRadians() - desiredYaw) < Constants.Swerve.yawTolerance
            && Math.abs(s_Shooter.getPitch() - desiredPitch) < Constants.Shooter.pitchTolerance
            && Math.abs(s_Shooter.getSpin() - desiredNoteVel) < Constants.Shooter.spinTolerance
            && kinesthetics.getVelocity().get(2, 0) < Constants.CommandConstants.speakerShooterOmegaMax;
    }

    @Override
    public void end(boolean interrupted) {
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
