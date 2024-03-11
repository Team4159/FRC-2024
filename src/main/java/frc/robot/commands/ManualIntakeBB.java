package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;

public class ManualIntakeBB extends Command {
    private Kinesthetics kinesthetics;
    private Shooter shooter;
    private Intake intake;
    public ManualIntakeBB(Kinesthetics k, Shooter sh, Intake i) {
        kinesthetics = k;
        shooter = sh;
        intake = i;
        // addCommands(
        //     sh.toPitch(Constants.Shooter.minimumPitch),
        //     sh.new ChangeNeck(SpinState.FW),
        //     i.new ChangeState(IntakeState.DOWN),
        //     new WaitUntilCommand(k::shooterHasNote).withTimeout(2)
        // );
        // // if the beambreak works, reverse the neck until it reopens
        // if (k.shooterHasNote()) {
        //     addCommands(
        //         sh.new ChangeNeck(SpinState.BW),
        //         new WaitUntilCommand(() -> !k.shooterHasNote()),
        //         sh.new ChangeNeck(SpinState.ST)
        //     );
        // }
        // addCommands(
        //     new ParallelCommandGroup( // ending commands
        //         i.new ChangeState(IntakeState.STOW),
        //         sh.new ChangeNeck(SpinState.ST)
        //     )
        // );

        addRequirements(kinesthetics, shooter, intake);
    }
    @Override 
    public void execute() {
        // CURRENTLY DOESNT WORK, COMMAND EXECUTES BUT BELOW LINES HAVE NO EFFECT
        shooter.toPitch(Constants.Shooter.minimumPitch);
        shooter.new ChangeNeck(SpinState.FW);
        intake.new ChangeState(IntakeState.DOWN);
    }

    @Override
    public boolean isFinished() {
        return kinesthetics.shooterHasNote();
    }

    @Override
    public void end(boolean interrupted) {
        intake.new ChangeState(IntakeState.STOW);
        if (interrupted) {
            shooter.new ChangeNeck(SpinState.ST);
        } else {
            shooter.new ChangeNeck(SpinState.BW).withTimeout(0.5);
        }
        super.end(interrupted);
    }
}
