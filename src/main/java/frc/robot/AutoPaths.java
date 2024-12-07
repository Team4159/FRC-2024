package frc.robot;

import java.util.Map;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.AutoCommand;
import frc.robot.Constants.SpinState;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.SpeakerLookupTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;

public class AutoPaths {
    public static Map<String, AutoCommand> autoMap = Map.of(
        "MID-4notecleanup", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> MID4notecleanup(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "MID-taxi", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> MIDtaxi(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "MID-3notefar", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> MID3notefar(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "SRC-2.5notefar", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> SRC3notefar(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "SRC-3notefar", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> SRC2notefar(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "SRC-hide", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> SRChide(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "AMP-disrupt", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> AMPdisrupt(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "AMP-5notecleanupfar", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> AMP5notecleanupfar(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "AMP-3.5notefar", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> AMP3notefar(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        //"setPoseAmp", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> setposeamp(autoFactory, kinesthetics, swerve, shooter, neck, intake),
        "spintest", (autoFactory, kinesthetics, swerve, shooter, neck , intake) -> testpath(autoFactory, kinesthetics, swerve, shooter, neck, intake)
    );
    public static Command betterTestAuto(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("betterTestAuto");
        final AutoTrajectory MIDtoC2 = factory.trajectory("MIDtoC2.traj", loop);
        final AutoTrajectory C2toMID = factory.trajectory("C2toMID.traj", loop);

        Pose2d startingPose = getInitialPose(MIDtoC2);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            new ParallelDeadlineGroup(
                MIDtoC2.cmd(),
                new IntakeAuto(kinesthetics, shooter, neck, intake, true)
            ).andThen(
                new SequentialCommandGroup(
                    new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0),
                    C2toMID.cmd()
                )
            )
        ));
        return loop.cmd();
    }
    public static Command MID4notecleanup(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("MID4notecleanup");
        final AutoTrajectory MIDtoC2 = factory.trajectory("MIDtoC2.traj", loop);
        final AutoTrajectory C2toC1 = factory.trajectory("C2toC1.traj", loop);
        final AutoTrajectory C1toC3 = factory.trajectory("C1toC3.traj", loop);

        Pose2d startingPose = getInitialPose(MIDtoC2);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(MIDtoC2, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C2toC1, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C1toC3, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ));
        return loop.cmd();
    }
    public static Command MID3notefar(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("MID3notefar");
        final AutoTrajectory MIDtoC2 = factory.trajectory("MIDtoC2.traj", loop);
        final AutoTrajectory C2toF3 = factory.trajectory("C2toF3.traj", loop);
        final AutoTrajectory F3toScore = factory.trajectory("F3toScore.traj", loop);

        Pose2d startingPose = getInitialPose(MIDtoC2);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(MIDtoC2, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C2toF3, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
            F3toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ));
        return loop.cmd();
    }
    public static Command MIDtaxi(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("MIDtaxi");
        final AutoTrajectory MIDtoC2 = factory.trajectory("MIDtoC2.traj", loop);

        Pose2d startingPose = getInitialPose(MIDtoC2);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(new ParallelCommandGroup(podiumShoot(kinesthetics, shooter, neck), intake.new ChangeState(IntakeState.DOWN, false, true)))
        .andThen(
            cmdWhileIntaking(MIDtoC2, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0))
        ));
        return loop.cmd();
    }
    public static Command SRC2notefar(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("SRC2noteFar");
        final AutoTrajectory SRCtoF4 = factory.trajectory("SRCtoF4.traj", loop);
        final AutoTrajectory F4toScore = factory.trajectory("F4toScore.traj", loop);
        final AutoTrajectory ScoretoF5 = factory.trajectory("ScoretoF5.traj", loop);

        Pose2d startingPose = getInitialPose(SRCtoF4);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(SRCtoF4, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
            F4toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(ScoretoF5, kinesthetics, swerve, shooter, neck, intake)
        ));
        return loop.cmd();
    }
    public static Command SRC3notefar(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("SRC3notefar");
        final AutoTrajectory SRCtoF4 = factory.trajectory("SRCtoF4.traj", loop);
        final AutoTrajectory F4toScore = factory.trajectory("F4toScore.traj", loop);
        final AutoTrajectory ScoretoF5 = factory.trajectory("ScoretoF5.traj", loop);
        final AutoTrajectory F5toScore = factory.trajectory("F5toScore.traj", loop);

        Pose2d startingPose = getInitialPose(SRCtoF4);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(SRCtoF4, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
                F4toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(ScoretoF5, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
            F5toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ));
        return loop.cmd();
    }
    public static Command SRChide(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("SRChide");
        final AutoTrajectory SRCtoHide = factory.trajectory("SRCtoHide.traj", loop);

        Pose2d startingPose = getInitialPose(SRCtoHide);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            SRCtoHide.cmd()
        ));
        return loop.cmd();
    }
    public static Command AMP3notefar(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("AMP3notefar");
        final AutoTrajectory AMPtoC1 = factory.trajectory("AMPtoC1.traj", loop);
        final AutoTrajectory C1toF2 = factory.trajectory("C1toF2.traj", loop);
        final AutoTrajectory F2toScore = factory.trajectory("F2toScore.traj", loop);
        final AutoTrajectory ScoretoF1 = factory.trajectory("ScoretoF1.traj", loop);

        Pose2d startingPose = getInitialPose(AMPtoC1);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(AMPtoC1, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C1toF2, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
            F2toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(ScoretoF1, kinesthetics, swerve, shooter, neck, intake)));
        return loop.cmd();
    }
    public static Command AMP5notecleanupfar(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("AMP5notecleanupfar");
        final AutoTrajectory AMPtoC1 = factory.trajectory("AMPtoC1.traj", loop);
        final AutoTrajectory C1toC2 = factory.trajectory("C1toC2.traj", loop);
        final AutoTrajectory C2toC3 = factory.trajectory("C2toC3.traj", loop);
        final AutoTrajectory C3toF3 = factory.trajectory("C3toF3.traj", loop);
        final AutoTrajectory F3toScore = factory.trajectory("F3toScore.traj", loop);

        Pose2d startingPose = getInitialPose(AMPtoC1);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            cmdWhileIntaking(AMPtoC1, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C1toC2, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C2toC3, kinesthetics, swerve, shooter, neck, intake)
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))
        ).andThen(
            cmdWhileIntaking(C3toF3, kinesthetics, swerve, shooter, neck, intake)
        ).andThen(
            F3toScore.cmd()
            .andThen(new SpeakerLookupTable(kinesthetics, swerve, shooter, neck, () -> 0, () -> 0, 2.0))));
        return loop.cmd();
    }
    public static Command AMPdisrupt(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("AMPdisrupt");
        final AutoTrajectory AMPtoDisrupt = factory.trajectory("AMPtoDisrupt", loop);

        Pose2d startingPose = getInitialPose(AMPtoDisrupt);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            AMPtoDisrupt.cmd()
        ));
        return loop.cmd();
    }
    public static Command TestCMD(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("TestCMD");
        final AutoTrajectory MIDtoC2 = factory.trajectory("MIDtoC2", loop);

        Pose2d startingPose = getInitialPose(MIDtoC2);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        //.andThen(podiumShoot(kinesthetics, shooter, neck))
        .andThen(
            MIDtoC2.cmd()
        ));
        return loop.cmd();
    }
    public static Command setposeamp(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("TestCMD");
        final AutoTrajectory AMPtoC1 = factory.trajectory("AMPtoC1", loop);

        Pose2d startingPose = getInitialPose(AMPtoC1);

        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose)));
        //.andThen(podiumShoot(kinesthetics, shooter, neck))
        return loop.cmd();
    }
    public static Command namedcommandstest(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("betterTestAuto");
        final AutoTrajectory traj = factory.trajectory("namedcommandstest.traj", loop);

        Pose2d startingPose = getInitialPose(traj);
        
        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(
            traj.cmd()
        ));
        return loop.cmd();
    }
    public static Command testpath(AutoFactory factory, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        final AutoLoop loop = factory.newLoop("betterTestAuto");
        final AutoTrajectory traj = factory.trajectory("spintest.traj", loop);

        Pose2d startingPose = getInitialPose(traj);
        
        loop.enabled().onTrue(new InstantCommand(() -> swerve.setPose(startingPose))
        .andThen(
            traj.cmd()
        )
        .andThen(
            swerve.runOnce(() -> swerve.setPose(traj.getFinalPose().orElse(new Pose2d())))
        ));
        return loop.cmd();
    }
    private static Pose2d getInitialPose(AutoTrajectory traj){
        return traj.getInitialPose().orElse(new Pose2d());
    }
    private static Command podiumShoot(Kinesthetics k, Shooter s, Neck n){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                n.new ChangeNeck(SpinState.ST),
                s.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, true)
                    .withTimeout(1)
            ),
            n.new ChangeNeck(k, SpinState.FW).raceWith(new WaitCommand(0.5)));
            //s.stopShooter());
    }
    //run an AutoTrajectory at the same time as IntakeAuto(without swerve movement). waits until the AutoTrajectory finished and gives 0.5 seconds for intake to finish after the trajectory
    public static Command cmdWhileIntaking(AutoTrajectory traj, Kinesthetics kinesthetics, CommandSwerveDrivetrain swerve, Shooter shooter, Neck neck, Intake intake){
        Command cmd = traj.cmd();
        return new ParallelCommandGroup(
            cmd,
            new SequentialCommandGroup(
                neck.new ChangeNeck(SpinState.FW),
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(kinesthetics::shooterHasNote),
                        neck.new ChangeNeck(SpinState.ST)
                    ),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(cmd::isFinished),
                        new WaitCommand(0.5)
                    )
                )
            )
        );
    }
}
