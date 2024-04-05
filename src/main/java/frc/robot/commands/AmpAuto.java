package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Deflector;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.ShooterCommand;
import frc.lib.math.RobotState;

public class AmpAuto extends ParallelCommandGroup {
    private static final ShooterCommand spinOnly = Constants.CommandConstants.ampShooterCommand.spinOnly();
    private static final ShooterCommand pitchOnly= new ShooterCommand(Constants.CommandConstants.ampShooterCommand.pitch(), null, null);

    public AmpAuto(Kinesthetics k, Swerve sw, Shooter sh, Neck n, Deflector d) {
        var all = DriverStation.getAlliance();
        if (all.isEmpty()) return;
        var desiredPose = Constants.Environment.amps.get(all.get());
        // offset the desired pose so the front of the robot touches the amp, not the center (which would be bad)
        desiredPose.plus(new Transform2d(-Constants.Swerve.trackWidth/2 - Constants.CommandConstants.bumperWidth, 0, new Rotation2d()));
        addCommands(
            new SwerveAuto(k, sw, new RobotState(desiredPose)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    sh.new ChangeState(pitchOnly),
                    new WaitUntilCommand(() ->
                        k.getPose().minus(desiredPose).getTranslation().getNorm() < Constants.CommandConstants.ampAutoDistanceToStartSpinning
                    )
                ),
                sh.new ChangeState(spinOnly),
                new ParallelDeadlineGroup(
                    n.new ChangeNeck(k, SpinState.FW),
                    d.new Raise()
                ),
                d.new Lower(),
                sh.stopShooter()
            )
        );
    }

    public static boolean isInRange(Kinesthetics k) { // are we close enough
        var all = DriverStation.getAlliance();
        if (all.isEmpty()) return false;
        return Constants.Environment.amps.get(all.get()).getTranslation()
            .minus(k.getPose().getTranslation()).getNorm()
            < Constants.CommandConstants.ampAutoDistanceMax;
    }
}
