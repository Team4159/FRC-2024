package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Shooter extends SubsystemBase {    
    private TalonFX angleMotorController;
    private MotionMagicDutyCycle angleDutyCycle;

    private CANSparkBase shooterMLeftController, shooterMRightController, neckMotorController;
    
    public Shooter() {
        angleMotorController = new TalonFX(Constants.Shooter.angleMotorIDs[0]);
        for (int n = 1; n < Constants.Shooter.angleMotorIDs.length; n++)
            new TalonFX(Constants.Shooter.angleMotorIDs[n]).setControl(new Follower(Constants.Shooter.angleMotorIDs[0], false));
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.follow(shooterMLeftController, true); // for now, no spin.
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getPosition().getValueAsDouble());
    }
    
    /** @param goalPitch radians */
    public void setGoalPitch(double goalPitch) {
        angleMotorController.setControl(angleDutyCycle.withPosition(Units.radiansToRotations(goalPitch)).withSlot(0));
    }

    /** @return radians / second */
    public double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

    /** @param goalNoteVel meters / second */
    public void setGoalSpin(double goalNoteVel) {
        shooterMLeftController.getPIDController().setReference(FeedForward.calculate(goalNoteVel), CANSparkBase.ControlType.kSmartVelocity);
    }

    public void stopSpin() {
        shooterMLeftController.stopMotor();
    }

    public void setNeck(SpinState ss) {
        neckMotorController.set(ss.multiplier * Constants.Shooter.neckSpeed);
    }

    private static class FeedForward { // TODO: Shooter feedforward
        static double calculate(double desiredNoteVelocity) { // meters / second -> rotations / minute
            return Conversions.MPSToRPS(desiredNoteVelocity, desiredNoteVelocity) / 60;
        }
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
