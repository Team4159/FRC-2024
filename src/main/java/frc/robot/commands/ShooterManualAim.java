package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterManualAim extends Command {
    private Shooter s_Shooter;
    private DoubleSupplier desiredPitch;
    
    public ShooterManualAim(Shooter s, DoubleSupplier pitchSupplier) {
        s_Shooter = s;
        desiredPitch = pitchSupplier;
    }

    @Override
    public void execute() {
        s_Shooter.setGoalPitch(desiredPitch.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Shooter.getPitch() - desiredPitch.getAsDouble()) < Constants.Shooter.pitchTolerance;
    }
}
