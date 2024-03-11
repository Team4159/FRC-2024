package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.util.Units;
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

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);

    /* Driver Buttons */
    private static final JoystickButton zeroGyro = new JoystickButton(driver, 1);

    private static final JoystickButton manualShoot = new JoystickButton(secondary, 1);
    private static final JoystickButton manualIntake = new JoystickButton(secondary, 2);
    private static final JoystickButton manualNeck = new JoystickButton(secondary, 3);
    private static final JoystickButton manualNeckBw = new JoystickButton(secondary, 4);
    private static final JoystickButton manualIntakeSpin = new JoystickButton(secondary, 5);
    private static final JoystickButton manualOuttake = new JoystickButton(secondary, 6);
    private static final JoystickButton manualAmp = new JoystickButton(secondary, 10);

    // private static final JoystickButton autoAmp = new JoystickButton(driver, 2);
    // private static final JoystickButton autoSpk = new JoystickButton(secondary, 8);
    // private static final JoystickButton autoIntake = new JoystickButton(secondary, 9);
    
    /* Subsystems */
    private static final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Deflector s_Deflector = new Deflector();

    public static final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);

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
        NamedCommands.registerCommand("ampAuto", new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Deflector));
        NamedCommands.registerCommand("intakeAuto", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake));
        NamedCommands.registerCommand("intakeStatic", new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true));
        NamedCommands.registerCommand("speakerSubwoofer", new SequentialCommandGroup(
            s_Shooter.new ChangeNeck(SpinState.ST),
            s_Shooter.new ChangeState(() -> new Pair<>(Constants.CommandConstants.speakerSubwooferPitch, Constants.CommandConstants.speakerSubwooferSpin), false),
            s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW),
            s_Shooter.new ChangeState(() -> new Pair<>(Constants.Shooter.minimumPitch, 0d))
        ));
        NamedCommands.registerCommand("speakerAutoAim", new ParallelCommandGroup(new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, null, null), new SpeakerAutoAimYaw(kinesthetics, s_Swerve, s_Shooter, null, null)));
        configureShooterMap();
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
   
        // only sets shooter and deflector to correct setpoint. in order to shoot, requires pressing manualNeck
        manualAmp.debounce(0.1)
            .onTrue(new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d, 0d))
            ))
            .whileTrue( new ParallelCommandGroup(
                s_Shooter.new ChangeState(() -> new ShooterCommand(
                    Constants.CommandConstants.ampShooterAngle,
                    Constants.CommandConstants.ampShooterSpin,
                    Constants.CommandConstants.ampShooterSpin
                ), true),
                s_Deflector.new Raise() // on end, resets pitch to 0, no need to reset pitch in onFalse block
            ))
            .onFalse(new ParallelCommandGroup(
                new InstantCommand( () -> s_Shooter.setNeckPercentage(SpinState.ST, 0)),
                s_Shooter.new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d, 0d))
            ));
        
        // uses beam break to stop note but does not use vision to move Swerve
        manualIntake.debounce(0.1)
            .whileTrue(new ParallelCommandGroup(
                /*originally new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Intake, true)*/
                s_Intake.new ChangeState(IntakeState.DOWN),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.FW)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Shooter.new ChangeNeck(kinesthetics, SpinState.ST)
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
            .whileTrue(new InstantCommand(() -> s_Shooter.setNeck(SpinState.FW, 0.5)))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));

        manualNeckBw.debounce(0.1)
            .whileTrue(s_Shooter.new ChangeNeck(SpinState.BW))
            .onFalse(s_Shooter.new ChangeNeck(SpinState.ST));

        manualShoot.debounce(0.1) 
            .onTrue(s_Shooter.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> new ShooterCommand(
                    Constants.Shooter.subwooferPitch,
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
            Commands.runOnce(() -> kinesthetics.setPose(Vision.getBotPose().toPose2d())),
            autoChooser.getSelected()
        );
    }

    public void configureShooterMap(){
        shooterTable.put(Units.inchesToMeters(51), 1.0);
    }
}
