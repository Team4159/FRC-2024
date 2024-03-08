package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private static final JoystickButton zeroGyro = new JoystickButton(driver, 5);
    private static final JoystickButton autoSpk = new JoystickButton(driver, 1);
    private static final JoystickButton autoAmp = new JoystickButton(driver, 2);
    // private static final JoystickButton autoIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualShoot = new JoystickButton(secondary, 1);
    private static final JoystickButton manualIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttake = new JoystickButton(secondary, 3);

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
                () -> driver.getY(), 
                () -> driver.getX(), 
                () -> driver.getZ(), 
                () -> false 
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(kinesthetics::zeroHeading));
        //uses lookup table
        autoSpk.debounce(0.3).and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
            .whileTrue(new SequentialCommandGroup(
                 s_Shooter.new ChangeNeck(SpinState.ST),
                 new ShooterLookupTable(kinesthetics, s_Shooter, s_Swerve),
                 s_Shooter.new ChangeNeck(SpinState.FW)
             )).onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        autoAmp.debounce(0.1).and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics))
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(new SequentialCommandGroup(
                new AmpAuto(kinesthetics, s_Swerve, s_Shooter),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
            ));
        // autoIntake.debounce(0.3).and(() -> !kinesthetics.shooterHasNote()) // && !kinesthetics.feederHasNote()
        //     .and(() -> IntakeAuto.canRun(kinesthetics))
        //     .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake))
        //     .onFalse(new ParallelCommandGroup(
        //         new InstantCommand(() -> s_Shooter.setNeck(SpinState.ST), s_Shooter),
        //         s_Intake.new ChangeState(IntakeState.STOW)
        // //     ));
        // manualShoot.debounce(0.1) // does not check if kinesthetics has note- because this should also work when kinesthetics fails
        //     .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
        //     .whileTrue(s_Shooter.new ChangeState(() -> new Pair<>(
        //             (1-secondary.getThrottle())/2 * Constants.CommandConstants.speakerShooterAngleMax + Constants.CommandConstants.speakerShooterAngleMin,
        //             Math.abs(secondary.getY()) * Constants.CommandConstants.shooterSpinMax
        //     ), true))
        //     .onFalse(new SequentialCommandGroup(
        //         new ParallelCommandGroup(
        //             s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW),
        //             new WaitCommand(1)
        //         ),
        //         s_Shooter.new ChangeNeck(SpinState.ST),
        //         s_Shooter.new ChangeState(() -> new Pair<>(0d, 0d))
        //     ));
        // temporary subwoofer shoot
        manualShoot.debounce(0.1) // does not check if kinesthetics has note- because this should also work when kinesthetics fails
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> new frc.lib.util.Triple<>(
                    1.00,
                    0.8 * Constants.CommandConstants.shooterSpinMax,
                    0.4 * Constants.CommandConstants.shooterSpinMax
            ), true))
            .onFalse(new SequentialCommandGroup(
                new ParallelCommandGroup(
                    s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW),
                    new WaitCommand(1)
                ),
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.new ChangeState(() -> new frc.lib.util.Triple<>(0d, 0d, 0d))
            ));
        manualIntake.debounce(0.1)
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Shooter.new ChangeNeck(SpinState.ST)
            ));
        manualOuttake.debounce(0.1)
            .whileTrue(s_Intake.new ChangeState(IntakeState.SPIT))
            .onFalse(s_Intake.new ChangeState(IntakeState.STOW));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> kinesthetics.setPose(Vision.getBotPose().toPose2d()))
        ); // add auto here
    }
}
