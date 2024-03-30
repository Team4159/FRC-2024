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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.commands.SpeakerAutoAim;

public class Swerve extends SubsystemBase {
    private Kinesthetics kinesthetics;

    private final SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    private Rotation2d driverAngleOffset = new Rotation2d(0);

    public void setKinesthetics(Kinesthetics k) {
        kinesthetics = k;
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
            Constants.Swerve.AutoConfig.pathFollower, // config, includes PID values
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red), // determines if autos should be flipped (i.e. if on Red Alliance)
            this // reference to this subsystem to set requirements
        );
        PPHolonomicDriveController.setRotationTargetOverride(() -> {
            var cmd = this.getCurrentCommand();
            if (cmd == null || !(cmd instanceof SpeakerAutoAim a) || a.latestYaw == null) return Optional.empty();
            return Optional.of(Rotation2d.fromRadians(a.latestYaw));
        });
        
        Timer.delay(0.1);
        for (SwerveModule mod : mSwerveMods) mod.resetToAbsolute();
    }

    /** @param rotation radians / second */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    kinesthetics.getRelativeHeading().plus(driverAngleOffset)
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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) states[mod.moduleNumber] = mod.getState();
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) positions[mod.moduleNumber] = mod.getPosition();
        return positions;
    }

    public void setAngleOffset() {
        driverAngleOffset = Rotation2d.fromRadians(-kinesthetics.getRelativeHeading().getRadians());
    }

    @Override
    public void periodic(){
        double[] drive = new double[mSwerveMods.length];
        double[] angle = new double[mSwerveMods.length];
        for(SwerveModule mod : mSwerveMods) {
            drive[mod.moduleNumber] = mod.getDriveCurrent();
            angle[mod.moduleNumber] = mod.getAngleCurrent();
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumberArray("Drive Current", drive);
        SmartDashboard.putNumberArray("Angle Current", angle);
    }

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
            var r = kinesthetics.getPose().getRotation().getRadians();
            var dr= MathUtil.angleModulus(desiredYaw.getAsDouble() - kinesthetics.getPose().getRotation().getRadians());
            drive(
                new Translation2d(passthroughTranslation.getAsDouble(), passthroughStrafe.getAsDouble()).times(Constants.Swerve.maxSpeed),
                Constants.CommandConstants.swerveYawPID.calculate(r, r+dr), true, false
            );
        }

        @Override
        public void end(boolean interrupted) {
            drive(
                new Translation2d(passthroughTranslation.getAsDouble(), passthroughStrafe.getAsDouble()).times(Constants.Swerve.maxSpeed),
                0, true, false
            );
        }

        @Override
        public boolean isFinished() {
            return MathUtil.isNear(desiredYaw.getAsDouble(), kinesthetics.getPose().getRotation().getRadians(), Constants.Swerve.yawTolerance);
        }
    }
}