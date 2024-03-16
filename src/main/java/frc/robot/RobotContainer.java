package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
    private static final JoystickButton resetGyro = new JoystickButton(driver, 1);
    private static final JoystickButton forceVision = new JoystickButton(driver, 10); // bottom left 

    private static final JoystickButton manualAmp = new JoystickButton(secondary, 3);
    private static final JoystickButton climb = new JoystickButton(secondary, 12);
    private static final JoystickButton manualShootPodium = new JoystickButton(secondary, 5);
    private static final JoystickButton manualShootSubwoofer = new JoystickButton(secondary, 4);
    private static final JoystickButton manualShootSourceIn = new JoystickButton(secondary, 6);
    private static final JoystickButton manualIntakeUp = new JoystickButton(secondary, 9);
    private static final JoystickButton manualIntakeDown = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttakeUp = new JoystickButton(secondary, 11);
    private static final JoystickButton manualOuttakeDown = new JoystickButton(secondary, 10);
    private static final JoystickButton manualFeed = new JoystickButton(secondary, 1); 

    private static final JoystickButton autoAmp = new JoystickButton(driver, 4);
    private static final JoystickButton autoSpk = new JoystickButton(driver, 3);
    // private static final JoystickButton autoIntake = new JoystickButton(driver, 14);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Deflector s_Deflector = new Deflector();
    private final Climber s_Climber = new Climber();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);
    @SuppressWarnings("unused")
    private final Vision s_Vision = new Vision(kinesthetics);

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

        configureAutoCommands();

        // Configure the button bindings
        configureButtonBindings();

        // configure SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser(); // can accept a default auto by passing in its name as a string
        SmartDashboard.putData("Auto Chooser", autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureAutoCommands() {
        // register Named Commands for PathPlanner, must be done before building autos
        NamedCommands.registerCommand("intakeStatic", new IntakeStatic(kinesthetics, s_Shooter, s_Intake));
        NamedCommands.registerCommand("speakerSubwoofer", new SequentialCommandGroup(
            s_Shooter.new ChangeNeck(SpinState.ST),
            s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, false),
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
        NamedCommands.registerCommand("speakerLookupTable", new ShooterLookupTable(kinesthetics, s_Shooter, s_Swerve, () -> 0, () -> 0));
        NamedCommands.registerCommand("ampAuto", new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector));
        NamedCommands.registerCommand("speakerAutoAim", new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> 0, () -> 0));
        NamedCommands.registerCommand("intakeAuto", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        resetGyro.onTrue(new InstantCommand(kinesthetics::zeroHeading));
        forceVision.onTrue(new InstantCommand(kinesthetics::forceVision));        

        // Automatic Command Groups
        autoSpk.and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(new SequentialCommandGroup(
                new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> -driver.getY(), () -> -driver.getX()),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
            )).onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        autoAmp.and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics))
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(new SequentialCommandGroup(
                new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
            )).onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        // autoIntake.and(() -> !kinesthetics.shooterHasNote()) // && !kinesthetics.feederHasNote()
        //     .and(() -> IntakeAuto.canRun(kinesthetics))
        //     .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake))
        //     .onFalse(new ParallelCommandGroup(
        //         s_Shooter.new ChangeNeck(SpinState.ST),
        //         s_Intake.new ChangeState(IntakeState.STOW)
        //     ));

        // Manual Command Groups
        manualAmp
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> Constants.CommandConstants.ampShooterCommand, true),
                s_Deflector.new Raise()
            ))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.stopShooter(),
                s_Deflector.new Lower()
            ));
        manualShootPodium
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, true))
            .onFalse(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSubwoofer
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, true))
            .onFalse(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSourceIn
            .whileTrue(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.BW, true),
                s_Shooter.new ChangeState(() -> Constants.CommandConstants.sourceInShooterCommand, false),
                new WaitUntilCommand(kinesthetics::shooterHasNote),
                new WaitCommand(0.05),
                s_Shooter.new ChangeNeck(SpinState.ST)
            ));
        manualFeed
            .whileTrue(s_Shooter.new ChangeNeck(SpinState.FW));
        manualIntakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.FW),
                s_Intake.new ChangeState(IntakeState.GARGLE)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW) 
            ));
        manualIntakeDown
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Shooter.new ChangeNeck(SpinState.ST)
            ));
        manualOuttakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.BW),
                s_Intake.new ChangeState(IntakeState.RETCH)
            ))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        manualOuttakeDown
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.BW),
                s_Intake.new ChangeState(IntakeState.SPIT)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));
        climb.whileTrue(s_Climber.new Raise());
    }

    public Command getTeleopInit() {
        var g = new ParallelCommandGroup(s_Shooter.new ChangeState(() -> Constants.Shooter.idleCommand, false));
        if (kinesthetics.getAlliance() == Alliance.Red)
            g.addCommands(Commands.runOnce(() -> kinesthetics.setHeading(
                kinesthetics.getHeading().minus(Rotation2d.fromDegrees(180))
            )));
        return g;
    }  

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
