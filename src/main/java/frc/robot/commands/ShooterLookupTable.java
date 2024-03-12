package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;

public class ShooterLookupTable extends SequentialCommandGroup{
    private Kinesthetics s_Kinesthetics;
    private Shooter s_Shooter;
    private Swerve s_Swerve;
    private double desiredPitch;

    public ShooterLookupTable(Kinesthetics k, Shooter s, Swerve sw){
        s_Kinesthetics = k;
        s_Shooter = s;
        s_Swerve = sw;
        desiredPitch = getBestCommand();
        addRequirements(s_Kinesthetics, s_Shooter, s_Swerve);
        addCommands(
            new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(desiredPitch, Constants.Shooter.leftSpeed, Constants.Shooter.rightSpeed), true),
                s_Swerve.new ChangeYaw(() -> s_Kinesthetics.getPose().getX(), () -> s_Kinesthetics.getPose().getY(), () -> getRequiredYaw(s_Kinesthetics)),
                new WaitUntilCommand(() -> s_Shooter.atState(desiredPitch, Constants.Shooter.leftSpeed, Constants.Shooter.rightSpeed))),
                s_Shooter.new ChangeNeck(SpinState.FW));
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    /** @return meters */
    private static double getDistance(Kinesthetics k){
        Transform3d dif = getDifference(k);
        double xDif = dif.getX();
        double yDif = dif.getY();
        double dist = Math.hypot(xDif, yDif);
        SmartDashboard.putNumber("Distance from speaker", dist);
        return dist;
    }

    /** @return required yaw in radians */
    private double getRequiredYaw(Kinesthetics k){
        Transform3d dif = getDifference(k);
        double xDif = dif.getX();
        double yDif = dif.getY();
        return Math.atan2(yDif, xDif);
    }

    /** @return shooter pitch */
    private double getBestCommand(){
        double closestMatch = 0;
        double secClosestMatch = 0;

        double closestAccuracy = Double.MAX_VALUE;
        double secClosestAccuracy = Double.MAX_VALUE;

        // Locate closest and second closest match
        for (double key : RobotContainer.shooterTable.keySet()) {
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
        return Constants.linearInterpolation(getDistance(s_Kinesthetics), closestMatch, secClosestMatch, RobotContainer.shooterTable.get(closestMatch), RobotContainer.shooterTable.get(secClosestMatch));
    }
}
