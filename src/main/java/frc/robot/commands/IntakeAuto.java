package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class IntakeAuto extends SequentialCommandGroup {
    public IntakeAuto(Kinesthetics k, Swerve sw, Shooter sh, Intake i) {
        super(
            // s.drive(new , 0, ); TODO angle towards note
            i.new ChangeState(IntakeState.DOWN),
            new ParallelCommandGroup(
                new WaitUntilCommand(k::feederHasNote),
                sh.new ChangeAim(() -> Constants.CommandConstants.shooterHandoffAngle)
            ),
            new ParallelCommandGroup(
                i.new ChangeState(IntakeState.HANDOFF),
                new InstantCommand(() -> sh.setNeck(SpinState.FW), sh)
            ).until(k::shooterHasNote)
        );
    }

    public static boolean isInRange(Kinesthetics k) {
        return false; // TODO is note close enough to grab
    }
}
