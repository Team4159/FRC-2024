package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {    
    private TalonFX angleMotorController;
    private MotionMagicDutyCycle angleDutyCycle;

    private CANSparkFlex shooterMLeftController, shooterMRightController;

    private CANSparkMax neckMotorController;
    
    public Shooter() {
        angleMotorController = new TalonFX(Constants.Shooter.angleMotorIDs[0]);
        for (int n = 1; n < Constants.Shooter.angleMotorIDs.length; n++)
            new TalonFX(Constants.Shooter.angleMotorIDs[n]).setControl(new Follower(Constants.Shooter.angleMotorIDs[0], false));
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.follow(shooterMLeftController, true); // for now, no spin.
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public double getPitch() {
        return angleMotorController.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    public void setGoalPitch(double goalRadians) {
        angleMotorController.setControl(angleDutyCycle.withPosition(goalRadians).withSlot(0));
    }

    public double getSpin() {
        return shooterMLeftController.getEncoder().getVelocity() * Math.PI / 30;
    }

    public void setGoalSpin(double goalRadiansPerSecond) { // TODO: figure out how to get radians per sec from m/s
        shooterMLeftController.getPIDController().setReference(goalRadiansPerSecond * 30/Math.PI, CANSparkMax.ControlType.kVelocity);
    }

    public void stopSpin() {
        shooterMLeftController.stopMotor();
    }

    public void setNeck(boolean stop, boolean isForwards) {
        neckMotorController.set((stop ? 0 : isForwards ? 1 : -1) * Constants.Shooter.neckSpeed);
    }

    public class ChangeAim extends Command {
        private DoubleSupplier desiredPitch;
        
        public ChangeAim(DoubleSupplier pitchSupplier) {
            addRequirements(Shooter.this);
            desiredPitch = pitchSupplier;
        }
    
        @Override
        public void execute() {
            setGoalPitch(desiredPitch.getAsDouble());
        }
    
        @Override
        public boolean isFinished() {
            return Math.abs(getPitch() - desiredPitch.getAsDouble()) < Constants.Shooter.pitchTolerance;
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted) setGoalPitch(Constants.Shooter.restingPitch);
            super.end(interrupted);
        }
    }
    
    public class ChangeSpin extends Command {
        private DoubleSupplier desiredSpin;
        
        public ChangeSpin(DoubleSupplier spinSupplier) {
            addRequirements(Shooter.this);
            desiredSpin = spinSupplier;
        }
    
        @Override
        public void execute() {
            setGoalSpin(desiredSpin.getAsDouble());
        }
    
        @Override
        public boolean isFinished() {
            return Math.abs(getSpin() - desiredSpin.getAsDouble()) < Constants.Shooter.spinTolerance;
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted) stopSpin();
            super.end(interrupted);
        }
    }
}
