package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
    private Kinesthetics kinesthetics;
    private Shooter s_Shooter;

    public SourceIntake(Kinesthetics k, Shooter sh) {
        kinesthetics = k;
        s_Shooter = sh;
        addRequirements(s_Shooter);
    }

    @Override
    public void execute() {
        s_Shooter.setGoalPitch(Constants.CommandConstants.sourceShooterIntakePitch);
        s_Shooter.setGoalSpin(Constants.CommandConstants.sourceShooterIntakeSpin);
        if (kinesthetics.shooterHasNote()) {
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.BW);
        } else {
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.ST);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            s_Shooter.setGoalPitch(Constants.Shooter.restingPitch);
            s_Shooter.stopSpin();
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.ST);
        }
        super.end(interrupted);
    }
}
