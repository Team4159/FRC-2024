package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.auto.SpeakerGetYaw;

public class Swerve extends SubsystemBase {
    private Kinesthetics kinesthetics;

        private SwerveModule[] mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    public Swerve() {
        // PathPlanner setup
        AutoBuilder.configureHolonomic(
            this.kinesthetics::getPose, // a supplier for the robot pose
            this.kinesthetics::setPose, // a consumer for the robot pose, accepts Pose2d
            () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates()), // a supplier for robot relative ChassisSpeeds
            (ChassisSpeeds chassisSpeeds) -> { // the drive method, accepts robot relative ChassisSpeeds
                SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                for(SwerveModule mod : mSwerveMods) mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
            }, 
            Constants.Swerve.autoPathFollowerConfig, // config, includes PID values
            () -> this.kinesthetics.getAlliance() == DriverStation.Alliance.Red, // determines if autos should be flipped (i.e. if on Red Alliance)
            this // reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public void setKinesthetics(Kinesthetics k) {
        kinesthetics = k;
        
        Timer.delay(0.1);
        resetModulesToAbsolute();
    }

    /** @param rotation radians / second */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    kinesthetics.getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods) mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }

    public void stop() {
        setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) states[mod.moduleNumber] = mod.getState();
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) positions[mod.moduleNumber] = mod.getPosition();
        return positions;
    }

    private void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods) mod.resetToAbsolute();
    }

    // override the path rotation if robot is currently shooting into speaker
    public Optional<Rotation2d> getRotationTargetOverride(){
        if (CommandScheduler.getInstance().isScheduled(SpeakerGetYaw.instance)) {
            // return an optional containing the speaker's rotation override (field relative rotation)
            return Optional.of(new Rotation2d(SpeakerGetYaw.instance.getDesiredYaw()));
        } else {
            // return an empty optional when path rotation should not be overriden
            return Optional.empty();
        }
    }

    // @Override
    // public void periodic(){
    //     for(SwerveModule mod : mSwerveMods){
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
    //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
    //     }
    // }

    public class ChangeYaw extends Command {
        private DoubleSupplier passthroughTranslation, passthroughStrafe, desiredYaw;

        public ChangeYaw(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier yaw) {
            passthroughTranslation = translation;
            passthroughStrafe = strafe;
            desiredYaw = yaw;
            addRequirements(Swerve.this);
        }

        @Override
        public void execute() {
            drive(
                new Translation2d(passthroughTranslation.getAsDouble(), passthroughStrafe.getAsDouble()).times(Constants.Swerve.maxSpeed),
                desiredYaw.getAsDouble(), true, false
            );
        }

        @Override
        public boolean isFinished() {
            return MathUtil.isNear(desiredYaw.getAsDouble(), kinesthetics.getHeading().getRadians(), Constants.Swerve.yawTolerance);
        }
    }
}