package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Swerve;

public class ShooterLookupTable extends ParallelCommandGroup {
    private static final Map<Double, Double> shooterTable = new HashMap<>() {{
        put(Units.inchesToMeters(51), 1.0);
    }}; // distance: pitch

    public ShooterLookupTable(Kinesthetics k, Shooter sh, Swerve sw, DoubleSupplier translationSup, DoubleSupplier strafeSup){
        super(
            sw.new ChangeYaw(translationSup, strafeSup, () -> getRequiredYaw(k).getRadians()),
            sh.new ChangeState(() -> new ShooterCommand(bestPitch(k), 30d, 15d), true)
        );
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    /** @return meters */
    private static double getDistance(Kinesthetics k){
        return getDifference(k).getTranslation().getNorm();
    }

    /** @return required yaw in radians */
    private static Rotation2d getRequiredYaw(Kinesthetics k){
        return getDifference(k).getTranslation().toTranslation2d().getAngle();
    }

    /** @return shooter pitch */
    private static double bestPitch(Kinesthetics k) {
        double distance = getDistance(k);

        double closestMatch = 0;
        double secClosestMatch = 0;

        double closestAccuracy = Double.MAX_VALUE;
        double secClosestAccuracy = Double.MAX_VALUE;

        // Locate closest and second closest match
        for (double key : shooterTable.keySet()) {
            double accuracy = Math.abs(key - distance);
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
