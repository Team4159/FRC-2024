package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;

public class ShooterLookupTable extends Command{
    private Kinesthetics s_Kinesthetics;
    private Shooter s_Shooter;
    private Swerve s_Swerve;

    public ShooterLookupTable(Kinesthetics k, Shooter s, Swerve sw){
        s_Kinesthetics = k;
        s_Shooter = s;
        s_Swerve = sw;
        addRequirements(s_Kinesthetics, s_Shooter, s_Swerve);
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    private static double getDistance(Kinesthetics k){
        Transform3d dif = getDifference(k);
        double xDif = dif.getX();
        double yDif = dif.getY();
        return Math.sqrt(Math.pow(xDif, 2) + Math.pow(yDif, 2));
    }

    /** @return radians */
    private double getYaw(Kinesthetics k){
        Transform3d dif = getDifference(k);
        double xDif = dif.getX();
        double yDif = dif.getY();
        return Math.atan(yDif / xDif);
    }

    @Override
    public void execute(){        
        s_Swerve.new ChangeYaw(null, null, () -> getYaw(s_Kinesthetics));
        s_Shooter.new ChangeState(() -> new frc.lib.util.Triple<>(getBestCommand(), 0.8, 0.4), true);
    }

    /** @return shooter pitch */
    private double getBestCommand(){
        double closestMatch = 0;
        double secClosestMatch = 0;

        double closestAccuracy = Double.MAX_VALUE;
        double secClosestAccuracy = Double.MAX_VALUE;

        // Locate closest and second closest match
        for (double key : Constants.Shooter.shooterTable.keySet()) {
            double accuracy = key;
            if(accuracy < closestAccuracy){
                secClosestMatch = closestMatch;
                closestMatch = key;
                secClosestAccuracy = closestAccuracy;
                closestAccuracy = accuracy;
            }
            else if(accuracy < secClosestAccuracy){
                secClosestMatch = key;
                secClosestAccuracy = accuracy;
            }
        }
        // Use linear interpolation on the two closest matches
        return Constants.linearInterpolation(getDistance(s_Kinesthetics), closestMatch, secClosestMatch, Constants.Shooter.shooterTable.get(closestMatch), Constants.Shooter.shooterTable.get(secClosestMatch));
    }
}
