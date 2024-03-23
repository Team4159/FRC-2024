package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.RobotState;
import frc.robot.Constants;

public class Kinesthetics extends SubsystemBase {
    // Subsystem Information
    private Swerve s_Swerve;

    // Sensor Information
    private Pigeon2 gyro;
    private DigitalInput shooterBeamBreak;

    // Data Fields
    private DriverStation.Alliance alliance;
    private SwerveDrivePoseEstimator poseEstimator;

    // Shuffleboard
    private final Field2d field = new Field2d();

    public Kinesthetics(Swerve s) {
        s_Swerve = s;
        s_Swerve.setKinesthetics(this);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        shooterBeamBreak = new DigitalInput(Constants.Shooter.beamBreakID);

        alliance = DriverStation.getAlliance().orElse(null);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
    
        ShuffleboardTab table = Shuffleboard.getTab("Kinesthetics");

        table.addBoolean("Shooter Note?", this::shooterHasNote);
        table.add("Pose Estimation", field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        var visionPose = Vision.getBotPose();
        if (visionPose != null)
            poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Vision.getLimelightPing());
        super.periodic();
        field.setRobotPose(getPose());
    }

    public void forceVision() {
        var visionPose = Vision.getBotPose();
        if (visionPose != null) setPose(visionPose.toPose2d());
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    // Public Getters & Setters
    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    public boolean shooterHasNote() {
        return !shooterBeamBreak.get();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public RobotState getRobotState() {
        var speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getHeading());
        return RobotState.fromVelocity(getPose(),
            -speeds.vyMetersPerSecond,
            speeds.vxMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }
}
