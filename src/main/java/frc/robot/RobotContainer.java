package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.autos.*;
import frc.robot.commands.ShooterManualAim;
import frc.robot.commands.ShooterManualSpin;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.groups.AmpAuto;
import frc.robot.commands.groups.SpeakerAutoAim;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private final JoystickButton autoSpk = new JoystickButton(driver, 1);
    private final JoystickButton autoAmp = new JoystickButton(driver, 2);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
    private final JoystickButton regurgitate = new JoystickButton(secondary, 3);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getY(), 
                () -> -driver.getZ(), 
                () -> -driver.getTwist(), 
                () -> true//robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> kinesthetics.zeroHeading()));
        autoSpk.debounce(0.3).and(kinesthetics::hasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
            .whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> s_Shooter.setNeck(true, false), s_Shooter),
                new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter),
                new InstantCommand(() -> s_Shooter.setNeck(false, true))
            )).onFalse(new InstantCommand(() -> s_Shooter.setNeck(true, true), s_Shooter));
        autoAmp.debounce(0.3).and(kinesthetics::hasNote)
            .whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> s_Shooter.setNeck(true, false), s_Shooter),
                new AmpAuto(kinesthetics, s_Swerve, s_Shooter),
                new InstantCommand(() -> s_Shooter.setNeck(false, true))
            )).onFalse(new InstantCommand(() -> s_Shooter.setNeck(true, true), s_Shooter));
        regurgitate // does not check if kinesthetics has note- because this should also work when kinesthetics fails
            .whileTrue(new ParallelCommandGroup(
                    new ShooterManualAim(s_Shooter, secondary::getY),
                    new ShooterManualSpin(s_Shooter, () -> Constants.CommandConstants.regurgitateSpeed)
                ).andThen(new InstantCommand(() -> s_Shooter.setNeck(false, true), s_Shooter))
            ).onFalse(new InstantCommand(() -> s_Shooter.setNeck(true, true), s_Shooter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> kinesthetics.setPose(new Pose2d()))
        ); // add auto here
    }
}
