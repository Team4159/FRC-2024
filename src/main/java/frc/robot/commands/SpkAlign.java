package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Swerve;

public class SpkAlign extends SequentialCommandGroup{
    private Kinesthetics k;
    private Swerve s;
    
    public SpkAlign(Kinesthetics k, Swerve s){
        this.k = k;
        this.s = s;
        addRequirements(k, s);
        addCommands(s.new ChangeYaw(() -> k.getPose().getX(), () -> k.getPose().getY(), () -> getRequiredYaw(k)));
    }

    private static Transform3d getDifference(Kinesthetics k) {
        return new Pose3d(k.getPose()).minus(Constants.Environment.speakers.get(k.getAlliance()));
    }

    /** @return required yaw in radians */
    private double getRequiredYaw(Kinesthetics k){
        Transform3d dif = getDifference(k);
        double xDif = dif.getX();
        double yDif = dif.getY();
        return Math.atan2(yDif, xDif);
    }
}
