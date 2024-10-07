package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.ChoreoAutoBindings;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private static final JoystickButton lookupTableShoot = new JoystickButton(driver, 5);
    private static final JoystickButton resetGyro = new JoystickButton(driver, 4);
    private static final JoystickButton forceVision = new JoystickButton(driver, 9);

    private static final JoystickButton manualAmp = new JoystickButton(secondary, 3);
    private static final JoystickButton manualShootSubwoofer = new JoystickButton(secondary, 4);
    private static final JoystickButton manualShootPodium = new JoystickButton(secondary, 5);
    private static final JoystickButton manualShootSourceIn = new JoystickButton(secondary, 6);
    private static final JoystickButton manualIntakeUp = new JoystickButton(secondary, 7);
    private static final JoystickButton manualIntakeDown = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttakeUp = new JoystickButton(secondary, 11);
    private static final JoystickButton manualOuttakeDown = new JoystickButton(secondary, 10);
    private static final JoystickButton manualClimberUp = new JoystickButton(secondary, 8);
    private static final JoystickButton manualClimberDown = new JoystickButton(secondary, 9);
    private static final Trigger manualFeed = new JoystickButton(driver, 1)
                                          .or(new JoystickButton(secondary, 1));

    //private static final JoystickButton autoAmp = new JoystickButton(driver, 4);
    //private static final JoystickButton autoSpk = new JoystickButton(driver, 3);
    private static final JoystickButton autoIntake = new JoystickButton(driver, 2);
    
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


    private final SendableChooser<String> autoChooser;

    private AutoFactory factory = Choreo.createAutoFactory(
        s_Swerve,
        kinesthetics::getPose,
        Constants.Swerve.AutoConfig.choreoController,
        (ChassisSpeeds speeds) -> { // needs to be robot-relative
            ChassisSpeeds reversedChassisSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(reversedChassisSpeeds);
            s_Swerve.setModuleStates(swerveModuleStates, false);
        },
        ()->{var ally = DriverStation.getAlliance(); return ally.isPresent() && ally.get().equals(Alliance.Red);},
        new ChoreoAutoBindings() // not useful until event markers
    );

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

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = getCommandChooser();
        SmartDashboard.putData("Autonomous Routine", autoChooser);
    }

    private SendableChooser<String> getCommandChooser(){
        SendableChooser<String> chooser = new SendableChooser<>();
        for(String name : AutoPaths.autoMap.keySet()){
            chooser.addOption(name, name);
        }
        return chooser;
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
        // autoSpk.and(kinesthetics::shooterHasNote).and(() -> SpeakerAutoAim.isInRange(kinesthetics))
        //     .onTrue(s_Neck.new ChangeNeck(SpinState.ST))
        //     .whileTrue(new SequentialCommandGroup(
        //         new SpeakerAutoAim(kinesthetics, s_Swerve, s_Shooter, () -> -driver.getY(), () -> -driver.getX()),
        //         s_Neck.new ChangeNeck(kinesthetics, SpinState.FW)
        //     )).onFalse(s_Neck.new ChangeNeck(SpinState.ST));
        // autoAmp.and(kinesthetics::shooterHasNote)//.and(() -> AmpAuto.isInRange(kinesthetics)) FIXME BROKEN LMAO
        //     .onTrue(s_Neck.new ChangeNeck(SpinState.ST))
        //     .whileTrue(new SequentialCommandGroup(
        //         new AmpAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Deflector),
        //         s_Neck.new ChangeNeck(kinesthetics, SpinState.FW)
        //     )).onFalse(s_Neck.new ChangeNeck(SpinState.ST));
        autoIntake.and(() -> !kinesthetics.shooterHasNote()).and(() -> IntakeAuto.canRun(kinesthetics))
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
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
        manualShootPodium // podium
            .onTrue(s_Neck.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerPodiumShooterCommand, true))
            .onFalse(new SequentialCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSubwoofer // subwoofer
            .onTrue(s_Neck.new ChangeNeck(SpinState.ST))
            .whileTrue(s_Shooter.new ChangeState(() -> Constants.CommandConstants.speakerSubwooferShooterCommand, true))
            .onFalse(new SequentialCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        lookupTableShoot 
            .onTrue(s_Neck.new ChangeNeck(SpinState.ST))
            .whileTrue(new SpeakerLookupTable(kinesthetics, s_Swerve, s_Shooter, () -> 0, () -> 0))
            .onFalse(new SequentialCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
                s_Shooter.stopShooter()
            ));
        manualShootSourceIn
            .whileTrue(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(kinesthetics::shooterHasNote),
                    s_Neck.new ChangeNeck(SpinState.BW, true),
                    s_Shooter.new ChangeState(Constants.CommandConstants.sourceInShooterCommand)
                ),
                new WaitCommand(0.05),
                s_Neck.new ChangeNeck(SpinState.ST)
            ));
        manualFeed
            .whileTrue(s_Neck.new ChangeNeck(SpinState.FW));
        manualIntakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.FW),
                s_Intake.new ChangeState(IntakeState.GARGLE)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
                s_Intake.new ChangeState(IntakeState.STOW) 
            ));
        manualIntakeDown
            .whileTrue(new IntakeAuto(kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake, true))
            .onFalse(new ParallelCommandGroup(
                s_Intake.new ChangeState(IntakeState.STOW),
                s_Neck.new ChangeNeck(SpinState.ST)
            ));
        manualOuttakeUp
            .whileTrue(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.BW),
                s_Intake.new ChangeState(IntakeState.RETCH)
            ))
            .onFalse(s_Neck.new ChangeNeck(SpinState.ST));
        manualOuttakeDown
            .whileTrue(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.BW),
                s_Intake.new ChangeState(IntakeState.SPIT)
            ))
            .onFalse(new ParallelCommandGroup(
                s_Neck.new ChangeNeck(SpinState.ST),
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
        return AutoPaths.autoMap.get(autoChooser.getSelected()).getCommand(factory, kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake);
    }
}
