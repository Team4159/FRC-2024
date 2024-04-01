package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;

public class ShooterIntaking extends SequentialCommandGroup {
    public ShooterIntaking(Kinesthetics k, Shooter sh) {
        addCommands(
            sh.toPitch(Constants.Shooter.minimumPitch),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote), //deadline
                sh.new ChangeNeck(SpinState.FW)
            ),
            sh.new ChangeNeck(SpinState.BW, true),
            new WaitCommand(0.02),
            sh.new ChangeNeck(SpinState.ST)
        );
    }
}
