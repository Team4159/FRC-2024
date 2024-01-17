package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kinesthetics extends SubsystemBase {
    // Subsystem Information
    private Swerve s_Swerve;

    // Sensor Information
    private Pigeon2 gyro;

    // Data Fields
    private DriverStation.Alliance alliance;
    private boolean rotationLock = false;
    private SwerveDrivePoseEstimator poseEstimator;

    public Kinesthetics(Swerve s) {
        s_Swerve = s;
        s_Swerve.setKinesthetics(this);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        alliance = DriverStation.getAlliance().orElse(null);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
    }

    @Override
    public void periodic() {
        poseEstimator.update(getHeading(), s_Swerve.getModulePositions());
        poseEstimator.addVisionMeasurement(Vision.getVisionPose().toPose2d(), Vision.latestLatency());
        super.periodic();
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    // Public Getters & Setters
    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    public boolean hasNote() {
        return true;
    }

    public boolean isRotationLocked() {
        return rotationLock;
    }

    public void lockRotation(boolean lock) {
        rotationLock = lock;
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

    public void zeroHeading(){
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
}
