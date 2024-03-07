package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private Kinesthetics kinesthetics;

    public SwerveModule[] mSwerveMods;

    public Swerve() {
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void setKinesthetics(Kinesthetics k) {
        kinesthetics = k;
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

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods) mod.resetToAbsolute();
    }

    @Override
    public void periodic(){
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
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
            drive(
                new Translation2d(passthroughTranslation.getAsDouble(), passthroughStrafe.getAsDouble()).times(Constants.Swerve.maxSpeed),
                desiredYaw.getAsDouble(), true, false
            );
        }

        @Override
        public boolean isFinished() {
            return Math.abs(kinesthetics.getHeading().getRadians() - desiredYaw.getAsDouble()) < Constants.Swerve.yawTolerance;
        }
    }
}