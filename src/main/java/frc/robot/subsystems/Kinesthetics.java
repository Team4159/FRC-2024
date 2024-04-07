package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.RobotState;
import frc.robot.Constants;

public class Kinesthetics extends SubsystemBase {
    // Subsystem Information
    private Swerve s_Swerve;

    // Sensor Information
    private Pigeon2 gyro;
    private DigitalInput shooterBeamBreak;
    private Debouncer shooterBeamBreakDebouncer = new Debouncer(0.03, Debouncer.DebounceType.kBoth);
    
    // Data Fields
    private SwerveDrivePoseEstimator poseEstimator;

    // Shuffleboard
    private final Field2d field = new Field2d();
    private final StructArrayPublisher<SwerveModuleState> swerveStates = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    public Kinesthetics(Swerve s) {
        s_Swerve = s;

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        shooterBeamBreak = new DigitalInput(Constants.Shooter.beamBreakID);

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d()
        );
    
        ShuffleboardTab table = Shuffleboard.getTab("Kinesthetics");

        table.addBoolean("Shooter Note?", this::shooterHasNote);
        table.add("Pose Estimation", field);

        s_Swerve.setKinesthetics(this);
    }

    @Override
    public void periodic() {
        double velocityOmega = gyro.getAngularVelocityZDevice().getValueAsDouble();
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        Vision.setRobotYaw(getPose().getRotation().getDegrees(), velocityOmega);
        if (Math.abs(velocityOmega) <= Constants.Environment.visionAngularCutoff) {
            var visionPose = Vision.getLimelightData();
            if (visionPose != null) poseEstimator.addVisionMeasurement(
                visionPose.pose().toPose2d(),
                Timer.getFPGATimestamp()-visionPose.ping(),
                VecBuilder.fill(visionPose.confidence(), visionPose.confidence(), 999)
            );
        }
        swerveStates.set(s_Swerve.getModuleStates());
        SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());
        field.setRobotPose(getPose());
    }

    public void forceVision() {
        var visionPose = Vision.getLimelightData();
        if (visionPose != null) setPose(visionPose.pose().toPose2d());
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    // Public Getters & Setters
    public boolean shooterHasNote() {
        return shooterBeamBreakDebouncer.calculate(!shooterBeamBreak.get());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
    }

    /** @return the rotation, inverted based on alliance (for driving) */
    public Rotation2d getRelativeHeading() {
        var r = getPose().getRotation();
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Blue)) {
            r = r.plus(Rotation2d.fromDegrees(180));
        }
        return r;
    }

    public RobotState getRobotState() {
        var speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());
        return RobotState.fromVelocity(getPose(),
            -speeds.vyMetersPerSecond,
            speeds.vxMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }
}
