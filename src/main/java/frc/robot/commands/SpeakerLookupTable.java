package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Swerve;

public class SpeakerLookupTable extends ParallelCommandGroup {
    public Double latestYaw = null;

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
        put(3.5, 0.56);
        put(4.0, 0.53);
        put(4.25, 0.495);
        put(4.5, 0.48);
        put(4.6, 0.475);
        put(4.8, 0.47);
        put(4.9, 0.465);
        put(5.0, 0.46);
    }}; // distance: pitch

    public SpeakerLookupTable(Kinesthetics k, Swerve sw, Shooter sh, Neck n, DoubleSupplier translationSup, DoubleSupplier strafeSup){
        this(k, sw, sh, n, translationSup, strafeSup, false);
    }
    public SpeakerLookupTable(Kinesthetics k, Swerve sw, Shooter sh, Neck n, DoubleSupplier translationSup, DoubleSupplier strafeSup, boolean continuous){
        double rootg = Math.sqrt(Constants.Environment.G);
        addCommands(
            sw.new ChangeYaw(translationSup, strafeSup, () -> -getDifference(k).toTranslation2d().getAngle().getRadians()),
            // sw.new ChangeYaw(translationSup, strafeSup, () -> {
            //     var transform = getDifference(k);
            //     var state = k.getRobotState();

            //     double roottwoh = Math.sqrt(2*transform.getZ()); // Z, up +
            //     boolean speakerIsOnRight = transform.getX() > 0;

            //     double relativex  = Math.abs(transform.getX()); // left+ right+
            //     double relativey  = (speakerIsOnRight ? -1 : 1) * transform.getY(); // forward backward
            //     double relativexv = (speakerIsOnRight ? 1 : -1) * state.getvx(); // speaker relative towards+ away-
            //     double relativeyv = (speakerIsOnRight ? -1 : 1) * state.getvy(); // speaker relative left- right+
                
            //     double n = relativex * rootg / roottwoh - relativexv;

            //     double desiredYaw = -Math.atan(- ((rootg * relativey) / roottwoh + relativeyv) / n); // CCW+, facing speaker = 0
            //     if (!speakerIsOnRight) desiredYaw += Math.PI; // flip it around

            //     SmartDashboard.putNumber("Kinesthetics yaw", k.getPose().getRotation().getDegrees());
            //     SmartDashboard.putNumber("Speaker absolute theta", Units.radiansToDegrees(desiredYaw)); // CCW+, 0 = North

            //     latestYaw = desiredYaw;
            //     return desiredYaw;
            // }),
            new SequentialCommandGroup(
                sh.new ChangeState(() -> new ShooterCommand(bestPitch(getDifference(k).toTranslation2d().getNorm()), 350d, 250d), continuous),
                new PrintCommand("lookuptable done"),
                n.new ChangeNeck(SpinState.FW),
                new WaitUntilCommand(() -> !k.shooterHasNote()),
                new ParallelCommandGroup(sh. new ChangeState(Constants.Shooter.idleCommand), n.new ChangeNeck(SpinState.ST), new InstantCommand(() -> sw.stop()))
            )
        );
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
        //Lookuptable best fit quadratic equation
        //return 0.0440779 * distance * distance - 0.434859 * distance + 1.55015;
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
