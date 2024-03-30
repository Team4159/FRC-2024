package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;

// this exists because IntakeAuto might require Swerve, which could mess up ParallelCommandGroups during auto paths
public class IntakeStatic extends SequentialCommandGroup {
    public IntakeStatic(Kinesthetics k, Shooter sh, Intake i) {
        addCommands(
            sh.toPitch(Constants.Shooter.minimumPitch),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote),
                sh.new ChangeNeck(SpinState.FW),
                i.new ChangeState(IntakeState.DOWN, true).withTimeout(4)
            ),
            sh.new ChangeNeck(SpinState.BW, true),
            new WaitCommand(0.02),
            sh.new ChangeNeck(SpinState.ST)
        );
    }
}
