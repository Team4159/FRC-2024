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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SpinState;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 16);
    // private final JoystickButton autoSpk = new JoystickButton(driver, 1);
    // private final JoystickButton autoAmp = new JoystickButton(driver, 2);
    // private final JoystickButton autoIntake = new JoystickButton(secondary, 2);
    private final JoystickButton manualIntake = new JoystickButton(secondary, 1);
    private final JoystickButton manualOuttake = new JoystickButton(secondary, 2);
    private final JoystickButton manualShoot = new JoystickButton(secondary, 3);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveManual(
                s_Swerve, 
                () -> -driver.getY(), 
                () -> -driver.getX(), 
                () -> -driver.getZ(), 
                () -> false //robotCentric.getAsBoolean()
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
        // autoSpk.debounce(0.3).and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
        //     .whileTrue(new SequentialCommandGroup(
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
        //         new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> -driver.getY(), () -> -driver.getX()),
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.FW))
        //     )).onFalse(new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter));
        // autoAmp.debounce(0.3).and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics))
        //     .whileTrue(new SequentialCommandGroup(
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
        //         new AmpAuto(kinesthetics, s_Swerve, s_Shooter),
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.FW))
        //     )).onFalse(new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter));
        // autoIntake.debounce(0.3).and(() -> !kinesthetics.shooterHasNote()) // && !kinesthetics.feederHasNote()
        //     .and(() -> IntakeAuto.canRun(kinesthetics))
        //     .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake))
        //     .onFalse(new ParallelCommandGroup(
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
        //         s_Intake.new ChangeState(IntakeState.STOW)
        // //     ));
        manualShoot.debounce(0.3) // does not check if kinesthetics has note- because this should also work when kinesthetics fails
            .whileTrue(new SequentialCommandGroup(
                new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
                s_Shooter.new ChangeState(
                    () -> (secondary.getThrottle()+1)/2 * Constants.CommandConstants.speakerShooterAngleMax,
                    () -> Math.abs(secondary.getY()) * Constants.CommandConstants.shooterSpinMax
                )
            )).onFalse(new SequentialCommandGroup(
                new InstantCommand(() -> s_Shooter.setNeck(SpinState.FW), s_Shooter),
                new WaitUntilCommand(() -> !kinesthetics.shooterHasNote()).withTimeout(2),
                new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter)
            ));
        manualIntake.debounce(0.3)
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));
        manualOuttake.debounce(0.3).whileTrue(s_Intake.new ChangeState(IntakeState.SPIT));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> kinesthetics.setPose(new Pose2d()))
        ); // add auto here
    }
}
