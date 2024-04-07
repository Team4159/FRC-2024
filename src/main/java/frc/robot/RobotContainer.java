package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private static final JoystickButton resetGyro = new JoystickButton(driver, 4);
    private static final JoystickButton forceVision = new JoystickButton(driver, 9);

    private static final JoystickButton manualAmp = new JoystickButton(secondary, 3);
    private static final JoystickButton manualShootPodium = new JoystickButton(secondary, 5);
    private static final JoystickButton manualShootSubwoofer = new JoystickButton(secondary, 4);
    private static final JoystickButton manualShootLob = new JoystickButton(secondary, 10);
    private static final JoystickButton manualShootSourceIn = new JoystickButton(secondary, 6);
    private static final JoystickButton manualIntakeUp = new JoystickButton(secondary, 7);
    private static final JoystickButton manualIntakeDown = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttakeUp = new JoystickButton(secondary, 11);
    private static final JoystickButton manualClimberUp = new JoystickButton(secondary, 8);
    private static final JoystickButton manualClimberDown = new JoystickButton(secondary, 9);
    private static final Trigger manualFeed = new JoystickButton(driver, 1)
                                          .or(new JoystickButton(secondary, 1));

    private static final JoystickButton autoAmp = new JoystickButton(driver, 3);
    private static final JoystickButton autoSpk = new JoystickButton(driver, 2);
    private static final JoystickButton autoIntake = new JoystickButton(driver, 5);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Neck s_Neck = new Neck();
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
                () -> -driver.getY(), 
                () -> -driver.getX(), 
                () -> driver.getZ(), 
                () -> false
            )
        );

        configureAutoCommands();

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Routine", autoChooser);
    }

    // register PathPlanner Commands, must be done before building autos
    private void configureAutoCommands() {
        NamedCommands.registerCommand("intakeStatic", new WrapperCommand(s_Intake.new ChangeState(IntakeState.DOWN)) {
            @Override
            public boolean isFinished() {return true;}
        }); // intake only intake
        NamedCommands.registerCommand("shooterSpinUp", 
            s_Shooter.new ChangeState(new Shooter.ShooterCommand(null, 450d, 325d))
        );
        NamedCommands.registerCommand("shooterIntaking", new SequentialCommandGroup( // intake without intake
            s_Shooter.new ChangeState(() -> new Shooter.ShooterCommand(Constants.Shooter.idleCommand.pitch(), null, null), false),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(kinesthetics::shooterHasNote),
                s_Neck.new ChangeState(SpinState.FW)
            ),
            s_Neck.new ChangeState(SpinState.BW, true),
            new WaitCommand(0.01),
            s_Neck.new ChangeState(SpinState.ST)
        ).withTimeout(5));

        NamedCommands.registerCommand("speakerSubwoofer", new SequentialCommandGroup(
            new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, true).withTimeout(2)
            ),
            s_Neck.new ChangeState(kinesthetics, SpinState.FW),
            s_Shooter.stopShooter().withTimeout(0.1)
        ));
        NamedCommands.registerCommand("speakerPodium", new SequentialCommandGroup(
            new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, true).withTimeout(2)
            ),
            s_Neck.new ChangeState(kinesthetics, SpinState.FW),
            s_Shooter.stopShooter().withTimeout(0.1)
        ));
        // NamedCommands.registerCommand("speakerLookupTable", new SequentialCommandGroup(
        //     new ParallelCommandGroup(
        //         s_Neck.new ChangeState(SpinState.ST),
        //         new SpeakerLookupTable(kinesthetics, s_Swerve, s_Shooter, () -> 0, () -> 0).withTimeout(2)
        //     ),
        //     s_Neck.new ChangeState(kinesthetics, SpinState.FW),
        //     s_Shooter.stopShooter().withTimeout(0.1)
        // ));
        NamedCommands.registerCommand("speakerAutoAim", new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> 0, () -> 0));

        NamedCommands.registerCommand("intakeAuto", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        /* Driver Buttons */
        resetGyro.onTrue(new InstantCommand(s_Swerve::setAngleOffset));
        forceVision.onTrue(new InstantCommand(kinesthetics::forceVision));

        // Automatic Command Groups
        autoSpk.and(kinesthetics::shooterHasNote).and(() -> SpeakerLookupTable.isInRange(kinesthetics))
            .onTrue(s_Neck.new ChangeState(SpinState.ST))
            .whileTrue(new SpeakerLookupTable(kinesthetics, s_Swerve, s_Shooter, () -> -driver.getY(), () -> -driver.getX()))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.stopShooter(),
                s_Neck.new ChangeState(SpinState.ST)
            ));
        autoAmp.and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics)) // FIXME BROKEN LMAO
            .onTrue(s_Neck.new ChangeState(SpinState.ST))
            .whileTrue(new SequentialCommandGroup(
                new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Deflector),
                s_Neck.new ChangeState(kinesthetics, SpinState.FW)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.stopShooter(),
                s_Neck.new ChangeState(SpinState.ST)
            ));
        autoIntake.and(() -> !kinesthetics.shooterHasNote()).and(() -> IntakeAuto.canRun(kinesthetics))
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));

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
            .onTrue(s_Neck.new ChangeState(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, true))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSubwoofer
            .onTrue(s_Neck.new ChangeState(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, true))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSourceIn
            .whileTrue(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(kinesthetics::shooterHasNote),
                    s_Neck.new ChangeState(SpinState.BW, true),
                    s_Shooter.new ChangeState(Constants.CommandConstants.sourceInShooterCommand)
                ),
                new WaitCommand(0.05),
                s_Neck.new ChangeState(SpinState.ST)
            ));
        manualShootLob
            .onTrue(s_Neck.new ChangeState(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(new Shooter.ShooterCommand(
                Units.degreesToRadians(45), 525d, 300d
            )))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualFeed
            .whileTrue(s_Neck.new ChangeState(SpinState.FW));
        manualIntakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.FW),
                s_Intake.new ChangeState(IntakeState.GARGLE)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW) 
            ));
        manualIntakeDown
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));
        manualOuttakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.BW),
                s_Intake.new ChangeState(IntakeState.RETCH)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeState(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));
        manualClimberUp
            .whileTrue(s_Climber.new ChangeState(SpinState.FW));
        manualClimberDown
            .whileTrue(s_Climber.new ChangeState(SpinState.BW));
    }

    public Command getTeleopInit() {
        return new ParallelCommandGroup(
            s_Shooter.new ChangeState(Constants.Shooter.idleCommand),
            s_Intake.new ChangeState(IntakeState.STOW)
        );
    }

    public Command getAutonomousCommand() {
        var a = autoChooser.getSelected();
        if (a == null) { // Sets the starting position of the robot
            a = new InstantCommand(() -> kinesthetics.setPose(
                DriverStation.getAlliance().map((all) -> all == DriverStation.Alliance.Red ?
                    new Pose2d(15.2, 5.53, Rotation2d.fromDegrees(0)) :
                    new Pose2d(1.31, 5.53, Rotation2d.fromDegrees(180))
                ).orElse(new Pose2d())
            ));
        }
        return a;
    }
}
