package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;

public class ShooterIntaking extends SequentialCommandGroup {
    public ShooterIntaking(Kinesthetics k, Shooter sh, Neck n) {
        addCommands(
            sh.stopShooter(),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote), //deadline
                n.new ChangeNeck(SpinState.FW)
            ),
            n.new ChangeNeck(SpinState.BW, true),
            new WaitCommand(0.01),
            n.new ChangeNeck(SpinState.ST)
        );
    }
}
