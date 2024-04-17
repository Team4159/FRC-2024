package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.Shooter.ShooterCommand;

public final class Constants {
    public static final double stickDeadband = 0.15;

    public static final class Swerve {
        public static final String canBus = "Drivetrain";
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3ANDAHALF);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75);
        public static final double wheelBase = Units.inchesToMeters(21.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        public static final double yawTolerance = Math.PI/4; // Radians

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 20;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.01;
        public static final int driveStatorCurrentLimit = 90;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.4;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.0;

        public static final class AutoConfig { //TODO: must be tuned to specific robot
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
            public static final double kPTransController = 5;
            public static final double kPRotatController = 1;
        
            /* Constraint for the motion profilied robot angle controller */
            public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

            // used by PathPlanner during setup
            public static final HolonomicPathFollowerConfig pathFollower = new HolonomicPathFollowerConfig( // TODO set values
                new PIDConstants(kPTransController, 0, 0), // translation PID constants
                new PIDConstants(kPRotatController, 0, 0), // rotation PID constants
                Constants.Swerve.maxSpeed,
                Constants.Swerve.wheelBase / Math.sqrt(2), // drive base radius in m
                new ReplanningConfig(true, true)
            );
        }

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-55.5);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-70); 

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-17.3);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.3); 

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Intake {
        public static final int angleMotorID = 1;
        public static final int intakeMotorID = 2;
        public static final int feederMotorID = 3;

        public static final double pitchTolerance = Math.PI/32; // radians
        public static final double spinTolerance = Math.PI/4; // radians

        public static final double intakeSpin = 0.7; // -1 to 1
        public static final double feederSpin = 0.45; // -1 to 1

        public static final PIDController anglePID = new PIDController(0.2, 0.0, 0.0);
        public static final double kG = 0.0;

        public static enum IntakeState {
            STOW(Units.rotationsToRadians(0.05), SpinState.ST), // starting pos & when moving
            GARGLE(Units.rotationsToRadians(0.05), SpinState.FW), // just move the motors
            DOWN(Units.rotationsToRadians(0.405), SpinState.FW), // intaking
            RETCH(Units.rotationsToRadians(0.05), SpinState.BW), // just move the motors
            SPIT(Units.rotationsToRadians(0.405), SpinState.BW); // outtaking

            public final double pitch;
            public final SpinState spin;
            /** @param p radians */
            private IntakeState(double p, SpinState s) {
                this.pitch = p;
                this.spin = s;
            }
        }

        public static final double intakeRange = 0.2; // meters
        public static final double intakeAngleRange = Units.degreesToRadians(64);

        public static final Pose3d luxonisTranslation = new Pose3d(
            new Translation3d(0, 0, Units.inchesToMeters(24)),
            new Rotation3d(Units.degreesToRadians(90), Units.degreesToRadians(26.57), Units.degreesToRadians(-90))
        ); // TODO: This must be tuned to specific robot
    } 

    public static final class Shooter {
        public static final int angleMotorID = 4;
        public static final int shooterMLeftID = 5;
        public static final int shooterMRightID = 6;
        public static final int neckMotorID = 7;
        public static final int beamBreakID = 0; // PWM

        public static final double pitchTolerance = Math.PI/64;
        public static final double spinTolerance = Math.PI/8;

        public static double pitchOffset = Units.degreesToRotations(-3);
        public static final double minimumPitch = Units.degreesToRadians(14);
        public static final double maximumPitch = Units.rotationsToRadians(0.2);
        public static final double neckSpeed = 0.60; // -1 to 1
        public static final ShooterCommand idleCommand = new ShooterCommand(minimumPitch, 200d);
        
        /** @param shooterSpinFF kS radians / second, kV radians / second per meter / second */
        public static final SimpleMotorFeedforward shooterSpinFF = new SimpleMotorFeedforward(-41.57843503917089, 28.371771957538527);
        
        public static final PIDController anglePID = new PIDController(0.75, 0.0003, 0.02);
        public static final double kG = 0.016;
    }

    public static final class Deflector {
        public static final int motorID = 9;

        public static final double maximumPitch = Units.rotationsToRadians(7.5);
    }

    public static final class Climber {
        public static final int motorLID = 10;
        public static final int motorRID = 11;
        public static final double climbSpeed = 0.8;

        // public static final double heightTolerance = Units.inchesToMeters(1);
        // public static final double maximumHeight = Units.inchesToMeters(15);
        // public static final double sprocketCircumference = Units.inchesToMeters(2 * Math.PI);
    }

    public static final class CommandConstants {
        public static final double bumperWidth = Units.inchesToMeters(2.75);

        public static final PIDController swerveYawPID = new PIDController(0.2, 0, 0.7) {{
            enableContinuousInput(-Math.PI, Math.PI);
            setTolerance(Constants.Swerve.yawTolerance);
        }};

        public static final ShooterCommand speakerPodiumShooterCommand = new ShooterCommand(
            0.7, 500d, 275d);
        public static final ShooterCommand speakerSubwooferShooterCommand = new ShooterCommand(
            1.1, 450d, 250d);

        public static final double ampAutoDistanceMax = 3.0; // meters
        public static final ShooterCommand ampShooterCommand = new ShooterCommand(
            Units.degreesToRadians(55), 130d);
        public static final double ampAutoDistanceToStartSpinning = 0.5; // meters

        public static final ShooterCommand sourceInShooterCommand = new ShooterCommand(
            Units.degreesToRadians(75), -90d);
    }

    public static final class Environment {
        /** @param speakers in meters */
        public static final Map<Alliance, Translation3d> speakers = Map.of(
            Alliance.Red, new Translation3d(16.579, 5.548, 1.891),
            Alliance.Blue, new Translation3d(-0.0381, 5.548, 1.891)
        );
        /** @param amps Pose2d is in meters */
        public static final Map<Alliance, Pose2d> amps = Map.of(
            Alliance.Red, new Pose2d(14.7008, 8.2042, Rotation2d.fromDegrees(90)),
            Alliance.Blue, new Pose2d(1.842, 8.2042, Rotation2d.fromDegrees(90))
        );
        /** @param G meters / second squared
         * Acceleration due to gravity
        */
        public static final float G = 9.8f;
        /** @param B ??? / ???
         * Some sort of aerodynamic constant
        */
        public static final double B = 0.096;
        /** @param visionAngularCutoff degrees / second
         * The maximum omega before vision is discarded
        */
        public static final double visionAngularCutoff = 720;
    }

    public static enum SpinState {
        FW(1), ST(0), BW(-1);
        public final int multiplier;
        private SpinState(int mult) {
            multiplier = mult;
        }
    }
}
