package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private static final Joystick driver = new Joystick(0);
    private static final Joystick secondary = new Joystick(1);
    private static final XboxController backupController = new XboxController(2);

    /* Driver Buttons */
    private static final JoystickButton lookupTableShoot = new JoystickButton(driver, 2);
    private static final Trigger manualIntakeDown = new JoystickButton(driver, 3)
                                                .or(new JoystickButton(secondary, 2))
                                                .or(new JoystickButton(backupController, 6));
    private static final JoystickButton forceVision = new JoystickButton(driver, 9);
    private static final JoystickButton resetGyro = new JoystickButton(driver, 4);

    private static final Trigger manualAmp = new JoystickButton(secondary, 3)
                                                .or(new JoystickButton(backupController, 2));
    private static final JoystickButton manualShootSubwoofer = new JoystickButton(secondary, 4);
    private static final Trigger manualShootPodium = new JoystickButton(secondary, 5)
                                                 .or(new JoystickButton(backupController, 1));
    private static final JoystickButton manualShootSourceIn = new JoystickButton(secondary, 6);
    private static final JoystickButton manualIntakeUp = new JoystickButton(secondary, 7);
    //private static final JoystickButton manualIntakeDown = new JoystickButton(secondary, 2);
    private static final JoystickButton manualOuttakeUp = new JoystickButton(secondary, 11);
    private static final JoystickButton manualOuttakeDown = new JoystickButton(secondary, 10);
    private static final Trigger manualClimberUp = new JoystickButton(secondary, 8)
                                               .or(new JoystickButton(backupController, 4));
    private static final Trigger manualClimberDown = new JoystickButton(secondary, 9)
                                                 .or(new JoystickButton(backupController, 3));
    private static final Trigger manualFeed = new JoystickButton(driver, 1)
                                          .or(new JoystickButton(secondary, 1))
                                          .or(new JoystickButton(backupController, 5));

    //private static final JoystickButton autoAmp = new JoystickButton(driver, 4);
    //private static final JoystickButton autoSpk = new JoystickButton(driver, 3);
    private static final JoystickButton autoIntake = new JoystickButton(driver, 2);
    
    /* Subsystems */
    private final CommandSwerveDrivetrain s_Swerve = TunerConstants.DriveTrain;
    private final Shooter s_Shooter = new Shooter();
    private final Neck s_Neck = new Neck();
    private final Intake s_Intake = new Intake();
    private final Deflector s_Deflector = new Deflector();
    private final Climber s_Climber = new Climber();

    private final Kinesthetics kinesthetics = new Kinesthetics(s_Swerve);
    @SuppressWarnings("unused")
    private final Vision s_Vision = new Vision(s_Swerve);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.maxSpeed * 0.1).withRotationalDeadband(Constants.Swerve.maxAngularVelocity * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

    private final SendableChooser<String> autoChooser;

    private AutoBindings autoBindings = createAutoBindings();

    private AutoFactory factory = Choreo.createAutoFactory(
        s_Swerve,
        s_Swerve::getPose,
        (Pose2d curPose, SwerveSample samples) -> { // needs to be robot-relative
            s_Swerve.followPath(curPose, samples);
        },
        ()->{var ally = DriverStation.getAlliance(); return ally.isPresent() && ally.get().equals(Alliance.Red);},
        new AutoBindings() // not useful until event markers
    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() -> drive.withVelocityX(MathUtil.applyDeadband(-driver.getY() * Constants.Swerve.maxSpeed, Constants.stickDeadband)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(MathUtil.applyDeadband(-driver.getX() * Constants.Swerve.maxSpeed, Constants.stickDeadband)) // Drive left with negative X (left)
            .withRotationalRate(MathUtil.applyDeadband(-driver.getZ() * Constants.Swerve.maxSpeed, Constants.stickDeadband)) // Drive counterclockwise with negative X (left)
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        autoChooser = getCommandChooser();
        SmartDashboard.putData("Autonomous Routine", autoChooser);
    }

    private AutoBindings createAutoBindings(){
        AutoBindings autoBindings = new AutoFactory.AutoBindings();
        autoBindings.bind("LookupTable", new SpeakerLookupTable(kinesthetics, s_Swerve, s_Shooter, s_Neck, null, null));
        return autoBindings;
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
        resetGyro.onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));
        //forceVision.onTrue(new InstantCommand(kinesthetics::forceVision));

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
            .whileTrue(new IntakeAuto(kinesthetics, s_Shooter, s_Neck, s_Intake))
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
            .whileTrue(new SpeakerLookupTable(kinesthetics, s_Swerve, s_Shooter, s_Neck, () -> 0, () -> 0))
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
            .whileTrue(new IntakeAuto(kinesthetics, s_Shooter, s_Neck, s_Intake, true))
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
        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getTeleopInit() {
        return new ParallelCommandGroup(
            s_Shooter.new ChangeState(Constants.Shooter.idleCommand),
            s_Intake.new ChangeState(IntakeState.STOW)
        );
    }
    public Command getAutonomousCommand() {
        System.out.println("getAutonomousCommand");
        if(autoChooser.getSelected() != null){
            return AutoPaths.autoMap.get(autoChooser.getSelected()).getCommand(factory, kinesthetics, s_Swerve, s_Shooter, s_Neck, s_Intake);
        }
        return new PrintCommand("No auto selected");
    }
}
