package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
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
    private Debouncer shooterBeamBreakDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    private AnalogGyro analogGyro = new AnalogGyro(0);
    private AnalogGyroSim gyroSim = new AnalogGyroSim(analogGyro);

    /** @param velocityOmega degrees / second */
    private StatusSignal<Double> velocityOmega;
    
    // Data Fields
    private SwerveDrivePoseEstimator poseEstimator;

    // Shuffleboard
    private final Field2d field = new Field2d();
    private final StructArrayPublisher<SwerveModuleState> swerveStates = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    public Kinesthetics(Swerve s) {
        s_Swerve = s;
        s_Swerve.setKinesthetics(this);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        shooterBeamBreak = new DigitalInput(Constants.Shooter.beamBreakID);

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d()
        );
    
        velocityOmega = gyro.getAngularVelocityZDevice();
        ShuffleboardTab table = Shuffleboard.getTab("Kinesthetics");

        table.addBoolean("Shooter Note?", this::shooterHasNote);
        table.add("Pose Estimation", field);
    }

    private double currentTime, prevTime;

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("dist from speaker", getDifference().toTranslation2d().getNorm());
        if(Constants.simulation){
            currentTime = Timer.getFPGATimestamp();
            double dts = currentTime - prevTime;
            prevTime = currentTime;
            ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(s_Swerve.getModuleStates());

            // finally adjust the simulator gyro.
            // the pose estimator figures out the X/Y part but it depends on the gyro.
            // since omega is the same in both coordinate schemes, just use that.
            double oldAngleDeg = gyroSim.getAngle();
            double dThetaDeg = -1.0 * new Rotation2d(speeds.omegaRadiansPerSecond * dts).getDegrees();
            double newAngleDeg = oldAngleDeg + dThetaDeg;
            // note that the "angle" in a gyro is NED, but everything else (e.g robot pose)
            // is NWU, so invert here.
            gyroSim.setAngle(newAngleDeg);
            poseEstimator.update(analogGyro.getRotation2d(), s_Swerve.getModulePositions());
        }
        else{
            poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
            Vision.setRobotYaw(getGyroYaw().getDegrees(), velocityOmega.getValueAsDouble());
            if (Math.abs(velocityOmega.getValueAsDouble()) <= Constants.Environment.visionAngularCutoff) {
                var visionPose = Vision.getLimelightData();
                if (visionPose != null)
                    poseEstimator.addVisionMeasurement(
                        visionPose.pose().toPose2d(),
                        Timer.getFPGATimestamp()-visionPose.ping(),
                        VecBuilder.fill(visionPose.confidence(), visionPose.confidence(), 99)
                    );
            }
        }
        swerveStates.set(s_Swerve.getModuleStates());
        SmartDashboard.putNumber("Rotation", getPose().getRotation().getRadians());
        field.setRobotPose(getPose());
        //field.getObject("speaker").setPose(new Pose2d(Constants.Environment.speakers.get(DriverStation.getAlliance().get()).toTranslation2d(), new Rotation2d()));
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
        s_Swerve.setAngleOffset();
    }

    public Rotation2d getHeading() {
        return getGyroYaw();
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
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroYaw());
        return RobotState.fromVelocity(getPose(),
            -speeds.vyMetersPerSecond,
            speeds.vxMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }
}
