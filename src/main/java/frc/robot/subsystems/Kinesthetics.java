package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kinesthetics extends SubsystemBase {
    // Subsystem Information
    private Swerve s_Swerve;

    // Sensor Information
    private Pigeon2 gyro;
    // private DigitalInput feederBeamBreak;
    private DigitalInput shooterBeamBreak;

    // Data Fields
    private DriverStation.Alliance alliance;
    private SwerveDrivePoseEstimator poseEstimator;

    public Kinesthetics(Swerve s) {
        s_Swerve = s;
        s_Swerve.setKinesthetics(this);

        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        Timer.delay(0.1);
        s_Swerve.resetModulesToAbsolute();

        // feederBeamBreak = new DigitalInput(Constants.Intake.beamBreakID);
        shooterBeamBreak = new DigitalInput(Constants.Shooter.beamBreakID);

        alliance = DriverStation.getAlliance().orElse(null);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("shoter", shooterBeamBreak.get());
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        var visionPose = Vision.getBotPose();
        if (visionPose != null)
            poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Vision.getLimelightPing());
        super.periodic();
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    // Public Getters & Setters
    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

    // public boolean feederHasNote() {
    //     return feederBeamBreak.get();
    // }

    public boolean shooterHasNote() {
        return shooterBeamBreak.get();
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

    /** @return v_x meters / second (forward +), v_y meters / second (left +), ω radians / second (ccw +)*/
    public Vector<N3> getVelocity() {
        var speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());
        var vec = new Vector<N3>(Nat.N3());
        vec.set(0, 0, speeds.vxMetersPerSecond);
        vec.set(1, 0, speeds.vyMetersPerSecond);
        vec.set(2, 0, speeds.omegaRadiansPerSecond);
        return vec;
    }
}
