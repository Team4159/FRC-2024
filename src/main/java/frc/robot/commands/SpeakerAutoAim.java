package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerAutoAim extends ParallelCommandGroup {
    public SpeakerAutoAim(Kinesthetics k, Swerve sw, Shooter sh, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        double rootg = Math.sqrt(Constants.Environment.G);
        addCommands(
            sh.new ChangeState(() -> {
                Transform3d transform = getDifference(k);

                double roottwoh = Math.sqrt(2*transform.getZ()); // Z, up +
                boolean speakerIsOnRight = transform.getY() > 0;

                double relativex  = Math.abs(transform.getY()); // left right
                double relativey  = (speakerIsOnRight ? -1 : 1) * transform.getX(); // forward backward
                double relativexv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(1, 0);
                double relativeyv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(0, 0);
                
                double n = relativex * rootg / roottwoh - relativexv;
                double m = relativey * rootg / roottwoh + relativeyv;

                double desiredPitch = Math.atan((roottwoh * rootg) / n);
                if (desiredPitch < 0) desiredPitch += Math.PI;
                double desiredNoteVel = Math.sqrt(
                    2*Constants.Environment.G*transform.getZ()
                    + n*n + m*m
                ) + Math.sqrt(Math.sqrt(
                    Constants.Environment.B * 54481/300000
                    * (
                        2 * Constants.Environment.G * transform.getZ()
                        + n*n + m*m
                    ) *
                    Math.sqrt(transform.getZ()*transform.getZ() + relativex * relativex + relativey * relativey)
                ) * 400/47 );

                return new Pair<>(desiredPitch, desiredNoteVel);
            }),
            sw.new ChangeYaw(translationSup, strafeSup, () -> {
                Transform3d transform = getDifference(k);

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
                return desiredYaw;
            })
        );
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    public static boolean isInRange(Kinesthetics k) {
        Translation2d offset = getDifference(k).getTranslation().toTranslation2d();
        return Math.abs(offset.getY()) < 5;
    }
}
