package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.RobotState;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class IntakeAuto extends SequentialCommandGroup {
    public IntakeAuto(Kinesthetics k, Swerve sw, Shooter sh, Intake i) {
        this(k, sw, sh, i, false);
    }

    public IntakeAuto(Kinesthetics k, Swerve sw, Shooter sh, Intake i, boolean disableMovement) {
        if (!disableMovement) {
            var notetrans = Vision.getNoteTranslation().toTranslation2d();
            if (notetrans.getNorm() > Constants.Intake.intakeRange || Math.abs(MathUtil.inputModulus(notetrans.getAngle().getDegrees(), -180, 180)) > Constants.Intake.intakeField/2) {
                addCommands(new SwerveAuto(k, sw, new RobotState(new Translation2d(notetrans.getNorm()-Constants.Swerve.trackWidth/2, notetrans.getAngle()), k.getHeading().plus(notetrans.getAngle()), new Vector<N3>(Nat.N3()))));
            }
        }
        addCommands(
            sh.toPitch(Constants.Shooter.minimumPitch),
            sh.new ChangeNeck(SpinState.FW),
            i.new ChangeState(IntakeState.DOWN),
            new WaitUntilCommand(k::shooterHasNote).withTimeout(2),
            new ParallelCommandGroup( // ending commands
                i.new ChangeState(IntakeState.STOW),
                sh.new ChangeNeck(SpinState.ST)
            )
        );
    }

    public static boolean canRun(Kinesthetics k) { // is there a note in view and does it seem close enough to grab
        var notetrans = Vision.getNoteTranslation().toTranslation2d();
        if (notetrans == null) return false;
        return notetrans.getNorm() < Constants.Intake.intakeRange*5;
    }
}
