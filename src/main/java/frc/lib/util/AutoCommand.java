package frc.lib.util;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

@FunctionalInterface
public interface AutoCommand{
    public Command getCommand(AutoFactory autoFactory, Kinesthetics kinesthetics, Swerve swerve, Shooter shooter, Neck neck, Intake intake);
}

