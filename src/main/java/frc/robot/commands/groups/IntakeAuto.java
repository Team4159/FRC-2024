package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Swerve;

public class IntakeAuto extends SequentialCommandGroup {
    public IntakeAuto(Kinesthetics k, Swerve s, Intake i) {
        super(
            // s.drive(new , 0, ); TODO angle towards note
            i.new ChangeState(IntakeState.DOWN),
            new WaitUntilCommand(k::feederHasNote),
            i.new ChangeState(IntakeState.STOW)
        );
    }

    public static boolean isInRange(Kinesthetics k) {
        return false; // TODO is note close enough to grab
    }
}
