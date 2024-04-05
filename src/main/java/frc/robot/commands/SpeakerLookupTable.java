package frc.robot.commands;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import javax.management.ConstructorParameters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Swerve;

public class SpeakerLookupTable extends ParallelCommandGroup {
    /** @see (distance (meters), pitch (radians)) */
    private static final Map<Double, Double> shooterTable = new HashMap<>() {{
        put(1.29, 1.1);
        put(1.5, 0.95);
        put(2.0, 0.85);
        put(2.5, 0.77);
        put(2.7, 0.69);
        put(2.8, 0.68);
        put(3.0, 0.63);
        put(3.25, 0.60);
        put(3.5, 0.54);
        put(4.0, 0.53);
        put(4.25, 0.495);
        put(4.5, 0.48);
        /*put(4.6, 0.49);
        put(4.8, 0.47);
        put(5.0, 0.46);*/
    }}; // distance: pitch

    public SpeakerLookupTable(Kinesthetics k, Shooter sh, Swerve sw, DoubleSupplier translationSup, DoubleSupplier strafeSup){
        k.forceVision();
        addCommands(
            sw.new ChangeYaw(translationSup, strafeSup, () -> getDifference(k).toTranslation2d().getAngle().getRadians()),
            new SequentialCommandGroup(
                sh.new ChangeState(() -> new ShooterCommand(bestPitch(getDifference(k).toTranslation2d().getNorm()), 500d, 375d), true)
                //sh.new ChangeNeck(SpinState.FW))
        ));
    }

    /** @return x right+, y forward+, z up+ */
    private static Translation3d getDifference(Kinesthetics k) {
        var all = DriverStation.getAlliance();
        if (all.isEmpty()) return null;
        var t2 = k.getPose().getTranslation();
        return Constants.Environment.speakers.get(all.get()).minus(new Translation3d(t2.getX(), t2.getY(), 0));
    }

    /** @return shooter pitch */
    private static double bestPitch(double distance) {
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
            closestAccuracy / (secClosestAccuracy + closestAccuracy)
        );
    }
}
