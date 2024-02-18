package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;

public class ShooterLookupTable extends Command{
    private Kinesthetics s_Kinesthetics;
    private Shooter s_Shooter;
    private Swerve s_Swerve;

    private DoubleSupplier desiredTranslation;
    private DoubleSupplier desiredStrafe;
    private double desiredYaw;
    private double desiredPitch;
    private double desiredSpin;

    public ShooterLookupTable(Kinesthetics k, Shooter s, Swerve sw, DoubleSupplier dt, DoubleSupplier ds){
        s_Kinesthetics = k;
        s_Shooter = s;
        s_Swerve = sw;
        desiredTranslation = dt;
        desiredStrafe = ds;
        addRequirements(s_Kinesthetics, s_Shooter, s_Swerve);
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(desiredTranslation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(desiredStrafe.getAsDouble(), Constants.stickDeadband);

        ShooterCommand sc = getBestCommand();

        s_Shooter.setGoalSpin(sc.speed());
        s_Shooter.setGoalPitch(sc.pitch());
        s_Swerve.drive(new Translation2d(translationVal, strafeVal), sc.yaw(), true, false);
    }

    private ShooterCommand getBestCommand(){
        Transform3d closestMatch = null;
        Transform3d secClosestMatch = null;

        // Locate closest and second closest match
        for (Transform3d key : Constants.Shooter.shooterTable.keySet()) {
            if(closestMatch == null){
                closestMatch = key;
            }
            else{
                double accuracy = Math.abs(key.getX() - s_Kinesthetics.getPose().getX()) + Math.abs(key.getY() - s_Kinesthetics.getPose().getX());
                double closestAccuracy = Math.abs(closestMatch.getX() - s_Kinesthetics.getPose().getX()) + Math.abs(closestMatch.getY() - s_Kinesthetics.getPose().getX());
                if(accuracy > closestAccuracy){
                    secClosestMatch = closestMatch;
                    closestMatch = key;
                }
            }
        }
        // Take average of closest match and second closest match
        double ds = (Constants.Shooter.shooterTable.get(closestMatch).speed() + Constants.Shooter.shooterTable.get(secClosestMatch).speed()) / 2;
        double dp = (Constants.Shooter.shooterTable.get(closestMatch).pitch() + Constants.Shooter.shooterTable.get(secClosestMatch).pitch()) / 2;
        double dy = (Constants.Shooter.shooterTable.get(closestMatch).yaw() + Constants.Shooter.shooterTable.get(secClosestMatch).yaw()) / 2;
        return new ShooterCommand(ds, dp, dy);
    }
}
