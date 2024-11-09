package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    Field2d field = new Field2d();
    ShuffleboardTab tab = Shuffleboard.getTab("Kinesthetics");

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final ApplyChassisSpeeds m_pathApplyChassisSpeeds = new ApplyChassisSpeeds();
    private final PIDController m_pathXController = new PIDController(1, 0, 0);
    private final PIDController m_pathYController = new PIDController(1, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(1, 0, 0);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add("Field", field);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add("Field", field);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public SwerveModuleState[] getModuleStates(){
        return m_moduleStates;
    }

    public void followPath(Pose2d pose, SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond = targetSpeeds.vxMetersPerSecond + m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond = targetSpeeds.vyMetersPerSecond + m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond = targetSpeeds.omegaRadiansPerSecond + m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyChassisSpeeds.withSpeeds(targetSpeeds)
        );
    }

    public Pose2d getPose(){
        if(getState().Pose == null){
            setPose(new Pose2d());
            return new Pose2d();
        }
        return getState().Pose;
    }

    public void setPose(Pose2d pose) {
        double angle = 0;
        if(DriverStation.getAlliance().orElse(null) == Alliance.Red){
            angle = Math.PI;
        }
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
        //s_Swerve.setAngleOffset(angle - pose.getRotation().getRadians());
    }

    // public SwerveModulePosition[] getModulePositions(){
    //     return new SwerveModulePosition()
    // }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        var visionPose = Vision.getLimelightData();
        if(visionPose != null) m_odometry
            .addVisionMeasurement(
                visionPose.pose().toPose2d(),
                Timer.getFPGATimestamp()-visionPose.ping(),
                VecBuilder.fill(visionPose.confidence(), visionPose.confidence(), 99));
        //System.out.println(getPigeon2().getAngle());
        for(int i = 0; i < 4; i++){
            SmartDashboard.putNumber("CANCoder " + i, getModule(i).getCANcoder().getPosition().getValueAsDouble());
            //System.out.println("Mod " + i + " position: " + getModule(i).getCANcoder().getPosition().getValueAsDouble());
        }
        field.setRobotPose(getPose());
    }
    public class ChangeYaw extends Command{
        DoubleSupplier desiredYaw;
        DoubleSupplier desiredTranslation;
        DoubleSupplier desiredStrafe;
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
        public ChangeYaw(DoubleSupplier desiredYaw, DoubleSupplier desiredTranslation, DoubleSupplier desiredStrafe){
            this.desiredYaw = desiredYaw;
            this.desiredTranslation = desiredTranslation;
            this.desiredStrafe = desiredStrafe;
            addRequirements(CommandSwerveDrivetrain.this);
        }

        @Override
        public void execute(){
            double desiredOmega = Constants.CommandConstants.swerveYawPID.calculate(
                getPose().getRotation().getRadians(),
                desiredYaw.getAsDouble());
            CommandSwerveDrivetrain.this.setControl(request
            .withVelocityX(desiredTranslation.getAsDouble())
            .withVelocityY(desiredStrafe.getAsDouble())
            .withRotationalRate(desiredOmega));
        }
        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return MathUtil.isNear(desiredYaw.getAsDouble(), getPose().getRotation().getRadians(), Constants.Swerve.yawTolerance);
        }
    }
}
