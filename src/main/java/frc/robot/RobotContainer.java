package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Intake.IntakeState;
import frc.robot.Constants.SpinState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ChangeNeck;

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private static final JoystickButton zeroGyro = new JoystickButton(driver, 1);
    private static final JoystickButton autoAmp = new JoystickButton(driver, 2);

    //private static final JoystickButton autoSpk = new JoystickButton(secondary, 1);
    //private static final JoystickButton autoIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualShoot = new JoystickButton(secondary, 1);
    private static final JoystickButton manualIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualIntakeSpin = new JoystickButton(secondary, 5);
    private static final JoystickButton manualNeck = new JoystickButton(secondary, 3);
        private static final JoystickButton manualNeckBw = new JoystickButton(secondary, 4);
    private static final JoystickButton manualOuttake = new JoystickButton(secondary, 0);
    private static final JoystickButton autoSpk = new JoystickButton(secondary, 8);
    private static final JoystickButton autoIntake = new JoystickButton(secondary, 9);
    private static final JoystickButton manualAmp = new JoystickButton(secondary, 10);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Deflector s_Deflector = new Deflector();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);

    public static final Map<Double, Double> shooterTable = new HashMap<>();

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
        configureShooterMap();
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
        // autoSpk.debounce(0.1).and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
        //     .whileTrue(new SequentialCommandGroup(
        //          s_Shooter.new ChangeNeck(SpinState.ST),
        //          new ShooterLookupTable(kinesthetics, s_Shooter, s_Swerve),
        //          s_Shooter.new ChangeNeck(SpinState.FW)
        //      )).onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        // autoAmp.debounce(0.1).and(kinesthetics::shooterHasNote).and(() -> AmpAuto.isInRange(kinesthetics))
        //     .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
        //     .whileTrue(new SequentialCommandGroup(
        //         new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector),
        //         s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
        //     ));
        /*autoIntake.debounce(0.1).and(() -> !kinesthetics.shooterHasNote()) // && !kinesthetics.feederHasNote()
            .and(() -> IntakeAuto.canRun(kinesthetics))
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake))
            .onFalse(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW)
            ));*/
        manualShoot.debounce(0.1) // does not check if kinesthetics has note- because this should also work when kinesthetics fails
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> new ShooterCommand(
                    (1-secondary.getThrottle())/2 * Constants.CommandConstants.speakerShooterAngleMax + Constants.CommandConstants.speakerShooterAngleMin,
                    Math.abs(secondary.getY()) * Constants.CommandConstants.shooterSpinMax,
                    Math.abs(secondary.getY()) * Constants.CommandConstants.shooterSpinMax
            ), true))
            .onFalse(new SequentialCommandGroup(
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW).withTimeout(1.5),
                s_Shooter.new ChangeNeck(SpinState.ST),
                s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d, 0d))
            ));
        manualAmp.debounce(0.05)
            .onTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d, 0d))
            ))
            .whileTrue( new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(
                    Constants.CommandConstants.ampShooterAngle,
                    Constants.CommandConstants.ampShooterSpin,
                    Constants.CommandConstants.ampShooterSpin
                ), true),
                s_Deflector.new Raise()
            ))
            .onFalse(new ParallelCommandGroup(
                //s_Deflector.new Raise(),
                //new InstantCommand( () -> s_Shooter.setNeckSpin(SpinState.FW, 0.7))
                new InstantCommand( () -> s_Shooter.setNeckPercentage(SpinState.ST, 0)),
                s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d, 0d))
            ));
        manualNeck.debounce(0.05)
            .whileTrue(new InstantCommand(() -> s_Shooter.setNeckPercentage(SpinState.FW, 0.7)))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        manualIntake.debounce(0.1)
            .whileTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeNeck(SpinState.FW),
                s_Intake.new ChangeState(IntakeState.DOWN)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Shooter.new ChangeNeck(SpinState.ST)
            ));
        manualIntakeSpin.debounce(0.1)
            .whileTrue(new ParallelCommandGroup(
                new InstantCommand(()-> s_Intake.setSpin(SpinState.FW)),
                s_Shooter.new ChangeNeck(SpinState.FW)
            ))
            .onFalse(new ParallelCommandGroup(
                new InstantCommand(()-> s_Intake.setSpin(SpinState.ST)),
                s_Shooter.new ChangeNeck(SpinState.ST))
            );
        manualOuttake.debounce(0.1)
            .whileTrue(s_Intake.new ChangeState(IntakeState.SPIT))
            .onFalse(s_Intake.new ChangeState(IntakeState.STOW));

        manualNeck.debounce(0.1)
            .whileTrue(new InstantCommand(() -> s_Shooter.setNeckCustomSpeed(SpinState.FW, 0.1)))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));
        manualNeckBw.debounce(0.1)
            .whileTrue(s_Shooter.new ChangeNeck(SpinState.BW))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));

        manualShoot.debounce(0.1) // does not check if kinesthetics has note- because this should also work when kinesthetics fails
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> new ShooterCommand(
                    //Constants.Shooter.subwooferPitch,
                    //for getting lookup table constants
                    (1 - secondary.getThrottle()) / 2 * Constants.Shooter.maximumPitch + Constants.Shooter.minimumPitch,
                    0.8 * Constants.CommandConstants.shooterSpinMax,
                    0.4 * Constants.CommandConstants.shooterSpinMax
                    ), true))
            .onFalse(
                new SequentialCommandGroup(
                    s_Shooter.new ChangeNeck(SpinState.ST),
                    s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0.05, 0.05)))
            );
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> kinesthetics.setPose(Vision.getBotPose().toPose2d()))
        ); // add auto here
    }

    public void configureShooterMap(){
        shooterTable.put(Units.inchesToMeters(51), 1.0);
    }
}
