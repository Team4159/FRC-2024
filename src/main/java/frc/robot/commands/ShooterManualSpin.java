package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterManualSpin extends Command {
    private Shooter s_Shooter;
    private DoubleSupplier desiredSpin;
    
    public ShooterManualSpin(Shooter s, DoubleSupplier spinSupplier) {
        s_Shooter = s;
        addRequirements(s_Shooter);
        desiredSpin = spinSupplier;
    }

    @Override
    public void execute() {
        s_Shooter.setGoalSpin(desiredSpin.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Shooter.getSpin() - desiredSpin.getAsDouble()) < Constants.Shooter.spinTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) s_Shooter.stopSpin();
        super.end(interrupted);
    }
}
