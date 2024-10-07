package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;

// this exists because IntakeAuto might require Swerve, which could mess up ParallelCommandGroups during auto paths
public class IntakeStatic extends SequentialCommandGroup {
    public IntakeStatic(Kinesthetics k, Intake i) {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote), // deadline
                i.new ChangeState(IntakeState.DOWN, true).withTimeout(0.5)
            )
        );
    }
}
