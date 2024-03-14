package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.auto.*;
import frc.robot.Constants.SpinState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShooterCommand;

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private static final JoystickButton zeroGyro = new JoystickButton(driver, 1);

    private static final JoystickButton manualAmp = new JoystickButton(secondary, 10);
    private static final JoystickButton manualShoot = new JoystickButton(secondary, 1);
    private static final JoystickButton manualIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttake = new JoystickButton(secondary, 7);

    private static final JoystickButton autoAmp = new JoystickButton(driver, 2);
    private static final JoystickButton autoSpk = new JoystickButton(secondary, 8);
    private static final JoystickButton autoIntake = new JoystickButton(secondary, 9);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Deflector s_Deflector = new Deflector();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);

    public static final Map<Double, Double> shooterTable = new HashMap<>();

    private final SendableChooser<Command> autoChooser;

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

        // register Named Commands for PathPlanner, must be done before building autos
        NamedCommands.registerCommand("intakeStatic", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true));
        NamedCommands.registerCommand("speakerSubwoofer", new SequentialCommandGroup(
            s_Shooter.new ChangeNeck(SpinState.ST),
            s_Shooter.new ChangeState(() -> new ShooterCommand(
                Constants.CommandConstants.speakerSubwooferPitch, 
                Constants.CommandConstants.speakerSubwooferSpin), 
                false),
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW),
            s_Shooter.stopShooter()
        ));
        NamedCommands.registerCommand("speakerPodium", new SequentialCommandGroup(
            s_Shooter.new ChangeNeck(SpinState.ST),
            s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, 
                false),
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW),
            s_Shooter.stopShooter()
        ));
        NamedCommands.registerCommand("speakerLookupTable", new ParallelCommandGroup(
            new ShooterLookupTable(kinesthetics, s_Shooter, s_Swerve, null, null), 
            new SpeakerGetYaw(kinesthetics, s_Swerve, s_Shooter, null, null)
        ));
        NamedCommands.registerCommand("ampAuto", new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector));
        NamedCommands.registerCommand("speakerAutoAim", new ParallelCommandGroup(
            new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, null, null), 
            new SpeakerGetYaw(kinesthetics, s_Swerve, s_Shooter, null, null)
        ));
        NamedCommands.registerCommand("intakeAuto", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake));

        // Configure the button bindings
        configureButtonBindings();

        // configure SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser(); // can accept a default auto by passing in its name as a string
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

        // Automatic Command Groups
        autoSpk.debounce(0.1).and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
            .whileTrue(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> -driver.getY(), () -> -driver.getX()),
                s_Shooter.new ChangeNeck(SpinState.FW)
            )).onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        autoAmp.debounce(0.1).and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics))
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(new SequentialCommandGroup(
                new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
            ));
        autoIntake.debounce(0.1).and(() -> !kinesthetics.shooterHasNote()) // && !kinesthetics.feederHasNote()
            .and(() -> IntakeAuto.canRun(kinesthetics))
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));

        // Manual Command Groups
        manualAmp.debounce(0.05)
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> Constants.CommandConstants.ampShooterCommand, true),
                s_Deflector.new Raise()
            ))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShoot.debounce(0.1)
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, true))
            .onFalse(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW).withTimeout(1),
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualIntake.debounce(0.1)
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Shooter.new ChangeNeck(SpinState.ST)
            ));
        manualOuttake.debounce(0.1)
            .onTrue(s_Intake.new ChangeState(IntakeState.SPIT))
            .onFalse(s_Intake.new ChangeState(IntakeState.STOW));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            //Commands.runOnce(() -> kinesthetics.setPose(Vision.getBotPose().toPose2d())),
            autoChooser.getSelected()
        ); // add auto here
    }
}
