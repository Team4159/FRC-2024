package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.robot.subsystems.Swerve;

public class SpeakerLTEquation extends ParallelCommandGroup {
    public Double latestYaw;

    public SpeakerLTEquation(Kinesthetics k, Swerve sw, Shooter sh, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
            sw.new ChangeYaw(translationSup, strafeSup, () -> latestYaw = getDifference(k).toTranslation2d().getAngle().getRadians()/* + Math.PI*/), // adding pi because back azimuth
            sh.new ChangeState(() -> new ShooterCommand(bestPitch(getDifference(k).toTranslation2d().getNorm()), 500d, 375d), true)
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
        return 0.0440779 * Math.pow(distance, 2) - 0.434859 * distance + 1.55015;
    }

    public static boolean isInRange(Kinesthetics k) {
        var isValid = Math.abs(getDifference(k).toTranslation2d().getNorm()) < 5;
        SmartDashboard.putBoolean("Can AutoAim", isValid);
        return isValid;
    }
}
