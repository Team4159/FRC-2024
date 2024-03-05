package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final String canBus = "Drivetrain";
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

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
        public static final double yawTolerance = Math.PI/256; // Radians

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 4.0; //TODO: This must be tuned to specific robot

        public static final class AutoConfig { //TODO: must be tuned to specific robot
            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
            public static final double kPXController = 1;
            public static final double kPYController = 1;
            public static final double kPThetaController = 1;
        
            /* Constraint for the motion profilied robot angle controller */
            public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-54);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-69); // -69.8

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-16);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55); // 55.4

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Intake {
        public static final int angleMotorID = 1;
        public static final int intakeMotorID = 2;
        public static final int feederMotorID = 3;
        public static final int beamBreakID = 1; // PWM

        public static final double pitchTolerance = Math.PI/64; // radians
        public static final double spinTolerance = Math.PI/16; // radians

        public static final double intakeSpin = 0.5; // -1 to 1
        public static final double feederSpin = 0.3; // -1 to 1

        public static final double intakeRange = 0.2; // meters
        public static final double intakeField = 64; // degrees

        public static enum IntakeState {
            STOW(Units.rotationsToRadians(0.012), SpinState.ST), // starting pos & when moving
            DOWN(Units.rotationsToRadians(0.465), SpinState.FW), // intaking
            SPIT(Units.rotationsToRadians(0.465), SpinState.BW); // outtaking

            public final double pitch;
            public final SpinState spin;
            /** @param p radians */
            private IntakeState(double p, SpinState s) {
                this.pitch = p;
                this.spin = s;
            }
        }

        public static final Translation3d luxonisTranslation = new Translation3d(); // TODO: This must be tuned to specific robot
    } 

    public static final class Shooter {
        public static final int angleMotorID = 4;
        public static final int shooterMLeftID = 5;
        public static final int shooterMRightID = 6;
        public static final int neckMotorID = 7;
        public static final int beamBreakID = 0; // PWM

        public static final double pitchTolerance = Math.PI/512;
        public static final double spinTolerance = Math.PI/256;

        public static final double pitchOffset = Units.degreesToRotations(-3);
        public static final double neckSpeed = 0.3; // volts / 12, -1 to 1
        
        /** @param shooterFeedForward kS radians / second, kV radians / second per meter / second */
        public static final SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(-41.57843503917089, 28.371771957538527);
    }

    public static final class CommandConstants {
        public static final double bumperWidth = Units.inchesToMeters(2.75);

        public static final double shooterSpinMax = 30; // meters / second
        public static final double shooterHandoffAngle = Units.rotationsToRadians(0.04);
        public static final double speakerShooterAngleMax = Units.rotationsToRadians(0.2);
        public static final double speakerShooterAngleMin = Units.rotationsToRadians(0.0);
        public static final double speakerAutoOmegaMax = Units.degreesToRadians(15);

        public static final double ampAutoDistanceMax = 3.0; // meters
        public static final double ampShooterAngle = Units.degreesToRadians(75);
        public static final double ampShooterSpin = 12;
        public static final double ampAutoDistanceToStartSpinning = 1; // meters
    }

    public static final class Environment {
        /** @param speakers Pose3d is in meters */
        public static final Map<Alliance, Pose3d> speakers = Map.of(
            Alliance.Red, new Pose3d(16.579, 5.548, 1.891, new Rotation3d(0, 0, Units.degreesToRadians(-90))),
            Alliance.Blue, new Pose3d(-0.0381, 5.548, 1.891, new Rotation3d(0, 0, Units.degreesToRadians(90)))
        );
        /** @param amps Pose2d is in meters */
        public static final Map<Alliance, Pose2d> amps = Map.of(
            Alliance.Red, new Pose2d(14.7008, 8.2042, Rotation2d.fromDegrees(0)),
            Alliance.Blue, new Pose2d(1.842, 8.2042, Rotation2d.fromDegrees(0))
        );
        /** @param G meters / second squared
         * Acceleration due to gravity
        */
        public static final float G = 9.8f;
        /** @param B ??? / ???
         * Some sort of aerodynamic constant
        */
        public static final double B = 0.096;
    }

    public static enum SpinState {
        FW(1), ST(0), BW(-1);
        public final int multiplier;
        private SpinState(int mult) {
            multiplier = mult;
        }
    }
}
