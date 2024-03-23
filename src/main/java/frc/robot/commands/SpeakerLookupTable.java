package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Swerve;

public class SpeakerLookupTable extends ParallelCommandGroup {
    private static final Map<Double, Double> shooterTable = new HashMap<>() {{
        put(Units.inchesToMeters(51), 1.0);
    }}; // distance: pitch

    public SpeakerLookupTable(Kinesthetics k, Shooter sh, Swerve sw, DoubleSupplier translationSup, DoubleSupplier strafeSup){
        super(
            sw.new ChangeYaw(translationSup, strafeSup, () -> getDifference(k).toTranslation2d().getAngle().getRadians()),
            sh.new ChangeState(() -> new ShooterCommand(bestPitch(getDifference(k).toTranslation2d().getNorm()), 450d, 350d), true)
        );
    }

    private static Translation3d getDifference(Kinesthetics k) {
        var t2 = k.getPose().getTranslation();
        return Constants.Environment.speakers.get(k.getAlliance()).minus(new Translation3d(t2.getX(), t2.getY(), 0));
    }

    /** @return shooter pitch */
    private static double bestPitch(double distance) {
        double closestMatch = 0;
        double secClosestMatch = 0;

        double closestAccuracy = Double.MAX_VALUE;
        double secClosestAccuracy = Double.MAX_VALUE;

        // Locate closest and second closest match
        for (double key : shooterTable.keySet()) {
            double accuracy = Math.abs( key - distance );
            if(accuracy < closestAccuracy) {
                secClosestMatch = closestMatch;
                closestMatch = key;
                secClosestAccuracy = closestAccuracy;
                closestAccuracy = accuracy;
            } else if (accuracy < secClosestAccuracy) {
                secClosestMatch = key;
                secClosestAccuracy = accuracy;
            }
        }
        // the more equal the accuracy, the more even the interpolation
        return MathUtil.interpolate(
            shooterTable.get(closestMatch),
            shooterTable.get(secClosestMatch),
            secClosestAccuracy / (secClosestAccuracy + closestAccuracy)
        );
    }
}
