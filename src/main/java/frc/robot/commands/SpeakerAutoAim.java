package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerAutoAim extends ParallelCommandGroup {
    public Double latestYaw;

    public SpeakerAutoAim(Kinesthetics k, Swerve sw, Shooter sh, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        double rootg = Math.sqrt(Constants.Environment.G);
        addCommands(
            sh.new ChangeState(() -> {
                var transform = getDifference(k);

                double roottwoh = Math.sqrt(2*transform.getZ()); // Z, up +
                boolean speakerIsOnRight = transform.getX() > 0;
                if (speakerIsOnRight != (k.getAlliance() == Alliance.Red)) {
                    System.out.println("UH OH!!!!");
                }

                double relativex  = Math.abs(transform.getX()); // left+ right+
                double relativey  = (speakerIsOnRight ? -1 : 1) * transform.getY(); // forward backward
                double relativexv = (speakerIsOnRight ? 1 : -1) * k.getVelocity().get(1, 0); // towards+ away-
                double relativeyv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(0, 0); 
                
                SmartDashboard.putNumber("Speaker dx", relativex);
                SmartDashboard.putNumber("Speaker dy", relativey);
                SmartDashboard.putNumber("Speaker dx/dt", relativexv);
                SmartDashboard.putNumber("Speaker dy/dt", relativeyv);

                double n = relativex * rootg / roottwoh - relativexv; // airtine
                double m = relativey * rootg / roottwoh + relativeyv;

                SmartDashboard.putNumber("Speaker airtime", n);

                double desiredPitch = Math.atan2(roottwoh * rootg, n);
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
                
                return new Shooter.ShooterCommand(
                    desiredPitch,
                    Constants.Shooter.shooterSpinFF.calculate(desiredNoteVel)
                );
            }, false),
            sw.new ChangeYaw(translationSup, strafeSup, () -> {
                var transform = getDifference(k);

                double roottwoh = Math.sqrt(2*transform.getZ()); // Z, up +
                boolean speakerIsOnRight = transform.getX() > 0;
                if (speakerIsOnRight != (k.getAlliance() == Alliance.Red)) {
                    System.out.println("UH OH!!!!");
                }

                double relativex  = Math.abs(transform.getX()); // left+ right+
                double relativey  = (speakerIsOnRight ? -1 : 1) * transform.getY(); // forward backward
                double relativexv = (speakerIsOnRight ? 1 : -1) * k.getVelocity().get(1, 0); // towards+ away-
                double relativeyv = (speakerIsOnRight ? -1 : 1) * k.getVelocity().get(0, 0); 
                
                double n = relativex * rootg / roottwoh - relativexv;

                double desiredYaw = -Math.atan(- ((rootg * relativey) / roottwoh + relativeyv) / n); // CCW+
                desiredYaw -= Math.PI / 2; // zero degrees is forwards, the equation assumes it's right
                if (!speakerIsOnRight) desiredYaw += Math.PI; // flip it around

                SmartDashboard.putNumber("Speaker absolute theta", Units.radiansToDegrees(desiredYaw)); // CCW+, 0 = North

                latestYaw = desiredYaw;
                return desiredYaw;
            })
        );
    }

    /** @return x right+, y forward+, z up+ */
    private static Translation3d getDifference(Kinesthetics k) {
        var t2 = k.getPose().getTranslation();
        return Constants.Environment.speakers.get(k.getAlliance()).minus(new Translation3d(-t2.getY(), t2.getX(), 0));
    }

    public static boolean isInRange(Kinesthetics k) {
        Translation2d offset = getDifference(k).toTranslation2d();
        return Math.abs(offset.getX()) < 5; // legal zone & limit of equations
    }
}
