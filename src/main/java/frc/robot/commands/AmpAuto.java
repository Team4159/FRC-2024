package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AmpAuto extends ParallelCommandGroup {
    public AmpAuto(Kinesthetics k, Swerve sw, Shooter sh) {
        var desiredPose = Constants.Field.amps.get(k.getAlliance());
        addCommands(
            new SwerveAuto(k, sw, desiredPose),
            new ParallelCommandGroup(
                sh.new ChangeAim(() -> Constants.CommandConstants.ampShooterAngle),
                new WaitUntilCommand(() ->
                    k.getPose().minus(desiredPose).getTranslation().getNorm() < Constants.CommandConstants.ampAutoDistanceToStartSpinning
                )
            ).andThen(sh.new ChangeSpin(() -> Constants.CommandConstants.ampShooterSpin))
        );
    }
}
