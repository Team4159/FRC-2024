package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class IntakeAuto extends SequentialCommandGroup {
    public IntakeAuto(Kinesthetics k, Shooter sh, Neck n, Intake i) {
        this(k, sh, n, i, false);
    }

    public IntakeAuto(Kinesthetics k, Shooter sh, Neck n, Intake i, boolean disableMovement) {
        addCommands(
            sh.stopShooter(), // return to initial angle
            new ParallelDeadlineGroup(
                new WaitUntilCommand(k::shooterHasNote),
                n.new ChangeNeck(SpinState.FW),
                i.new ChangeState(IntakeState.DOWN)
            ),
            n.new ChangeNeck(SpinState.BW, true),
            new WaitCommand(0.02),
            n.new ChangeNeck(SpinState.ST)
        );
    }

    public static boolean canRun(Kinesthetics k) { // is there a note in view and does it seem close enough to grab
        var notetrans = Vision.getNoteTranslation();
        if (notetrans == null) return false;
        return notetrans.toTranslation2d().getNorm() < Constants.Intake.intakeRange*5;
    }
}
