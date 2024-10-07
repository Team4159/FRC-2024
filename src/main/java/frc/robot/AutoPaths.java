package frc.robot;

import java.util.Map;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.AutoCommand;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.commands.SpeakerLookupTable;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;

public class AutoPaths {
    public static Map<String, AutoCommand> autoMap = Map.of(
        "betterTestAuto", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> betterTestAuto(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "testAuto", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> testAuto(autoFactory, kinesthetics, swerve, shooter, neck, intake)
    );

    public static Command testAuto(AutoFactory factory, Kinesthetics k, Swerve sw, Shooter sh, Neck n, Intake i){
        final AutoLoop loop = factory.newLoop("testAuto");
        final AutoTrajectory SPKtoC2 = factory.trajectory("SPKtoC2SPN", loop);
        final AutoTrajectory C2toSPK = factory.trajectory("C2toSPKSPN", loop);

        Pose2d startingPose;
        if (SPKtoC2.getInitialPose().isPresent()) {
            startingPose = SPKtoC2.getInitialPose().get();
            k.setPose(startingPose);
        } else {
            startingPose = new Pose2d();
        }
        
        loop.enabled().onTrue(new InstantCommand(() -> k.setPose(startingPose))
        .andThen(
            new ParallelCommandGroup(
                i.new ChangeState(IntakeState.DOWN, true),
                SPKtoC2.cmd()
            )
        ));

        SPKtoC2.done()
        .onTrue(new SequentialCommandGroup(
            new ParallelRaceGroup(new SpeakerLookupTable(k, sw, sh, () -> 0, () -> 0), new WaitCommand(2)),
            new ParallelCommandGroup(new InstantCommand(() -> sh.stopShooter()), new InstantCommand(() -> sw.stop())),
            C2toSPK.cmd()));

        return loop.cmd();
    }
    public static Command betterTestAuto(AutoFactory factory, Kinesthetics k, Swerve sw, Shooter sh, Neck n, Intake i){
        final AutoLoop loop = factory.newLoop("betterTestAuto");
        final AutoTrajectory SPKtoC2 = factory.trajectory("SPKtoC2", loop);
        final AutoTrajectory C2toSPK = factory.trajectory("C2toSPK", loop);

        Pose2d startingPose;
        if (SPKtoC2.getInitialPose().isPresent()) {
            startingPose = SPKtoC2.getInitialPose().get();
            k.setPose(startingPose);
        } else {
            startingPose = new Pose2d();
        }
        
        loop.enabled().onTrue(new InstantCommand(() -> k.setPose(startingPose))
        .andThen(
            new ParallelCommandGroup(
                i.new ChangeState(IntakeState.DOWN, true),
                SPKtoC2.cmd()
            )
        ));

        SPKtoC2.done()
        .onTrue(new SequentialCommandGroup(
            new ParallelRaceGroup(new SpeakerLookupTable(k, sw, sh, () -> 0, () -> 0), new WaitCommand(2)),
            new ParallelCommandGroup(new InstantCommand(() -> sh.stopShooter()), new InstantCommand(() -> sw.stop())),
            C2toSPK.cmd()));

        return loop.cmd();
    }

}
