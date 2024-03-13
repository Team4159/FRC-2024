package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterCommand;

public class SourceIntakeManual extends Command {
    private Kinesthetics kinesthetics;
    private Shooter s_Shooter;

    public SourceIntakeManual(Kinesthetics k, Shooter sh) {
        kinesthetics = k;
        s_Shooter = sh;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute() {
        s_Shooter.new ChangeState(() -> new ShooterCommand(
            Constants.CommandConstants.sourceShooterIntakePitch, 
            Constants.CommandConstants.sourceShooterIntakeSpin), 
            true);
        if (kinesthetics.shooterHasNote()) {
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.BW);
        } else {
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.ST);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            s_Shooter.stopShooter();
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.ST);
        }
        super.end(interrupted);
    }
}
