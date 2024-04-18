package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.RobotState;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class IntakeAuto extends SequentialCommandGroup {
    public IntakeAuto(Kinesthetics k, Swerve sw, Shooter sh, Neck n, Intake i) {
        this(k, sw, sh, n, i, null, false);
    }

    public IntakeAuto(Kinesthetics k, Swerve sw, Shooter sh, Neck n, Intake i, Vision v, boolean disableMovement) {
        var notetrans3d = Vision.getNoteTranslation();
        if (!disableMovement && notetrans3d != null) {
            Translation2d notetrans = notetrans3d.toTranslation2d();
            if (notetrans.getNorm() > Constants.Intake.intakeRange || Math.abs(MathUtil.inputModulus(notetrans.getAngle().getRadians(), -Math.PI/2, Math.PI)) > Constants.Intake.intakeAngleRange) {
                Rotation2d noteAngle = k.getPose().getRotation().plus(notetrans.getAngle());
                addCommands(new SwerveAuto(k, sw, new RobotState(new Translation2d(notetrans.getNorm()-Constants.Swerve.wheelBase/2, noteAngle), noteAngle, new Vector<N3>(Nat.N3()))));
            }
        }
        addCommands(
            sh.new ChangeState(new Shooter.ShooterCommand(Constants.Shooter.idleCommand.pitch(), null)), // return to initial angle
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote),
                n.new ChangeState(SpinState.FW),
                i.new ChangeState(IntakeState.DOWN),
                sh.stopShooter()
            )
        );
        if (v != null) addCommands(v.new Flash());
        addCommands(
            n.new ChangeState(SpinState.BW, true),
            new WaitCommand(0.02),
            n.new ChangeState(SpinState.ST)
        );
    }

    public static boolean canRun(Kinesthetics k) { // is there a note in view and does it seem close enough to grab
        var notetrans = Vision.getNoteTranslation();
        if (notetrans == null) return false;
        return notetrans.toTranslation2d().getNorm() < Constants.Intake.intakeRange*5;
    }
}
