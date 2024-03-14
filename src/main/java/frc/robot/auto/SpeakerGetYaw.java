package frc.robot.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerGetYaw extends Command {
    public static SpeakerGetYaw instance;
    public double OverrideYaw;

    public SpeakerGetYaw(Kinesthetics k, Swerve sw, Shooter sh, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        double rootg = Math.sqrt(Constants.Environment.G);
        var transform = getDifference(k);

        double roottwoh = Math.sqrt(2*transform.getZ()); // Z, up +
        boolean speakerIsOnRight = transform.getY() > 0;

        double relativex  = Math.abs(transform.getY()); // left right
        double relativey  = (speakerIsOnRight ? -1 : 1) * transform.getX(); // forward backward
        double relativexv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(1, 0);
        double relativeyv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(0, 0);
        
        double n = relativex * rootg / roottwoh - relativexv;

        double desiredYaw = Math.atan(- ((rootg * relativey) / roottwoh + relativeyv) / n);
        desiredYaw -= Math.PI / 2; // zero degrees is forwards, the equation assumes it's right
        if (!speakerIsOnRight) desiredYaw += Math.PI; // flip it around
        OverrideYaw = desiredYaw;
    }

    public double getDesiredYaw() {
        return OverrideYaw;
    }

    private static Translation3d getDifference(Kinesthetics k) {
        var t2 = k.getPose().getTranslation();
        return Constants.Environment.speakers.get(k.getAlliance()).minus(new Translation3d(t2.getX(), t2.getY(), 0));
    }

    public static boolean isInRange(Kinesthetics k) {
        Translation2d offset = getDifference(k).toTranslation2d();
        return Math.abs(offset.getY()) < 5;
    }
}
