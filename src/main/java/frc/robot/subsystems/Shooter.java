package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
    private CANSparkBase angleMotorController, shooterMLeftController, shooterMRightController, neckMotorController;
    
    public Shooter() {
        angleMotorController = new CANSparkMax(Constants.Shooter.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.follow(shooterMLeftController, true); // for now, no spin.
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getEncoder().getPosition());
    }
    
    /** @param goalPitch radians */
    public void setGoalPitch(double goalPitch) {
        angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kSmartMotion);
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

    @Override
    public void periodic() {
        System.out.println(getPitch());
    }

    private static class FeedForward { // TODO: Shooter feedforward
        static double calculate(double desiredNoteVelocity) { // meters / second -> rotations / minute
            return Conversions.MPSToRPS(desiredNoteVelocity, desiredNoteVelocity) / 60;
        }
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> pitch, null);
    }

    public ChangeState toSpin(double spin) {
        return new ChangeState(null, () -> spin);
    }

    public class ChangeState extends Command {
        private DoubleSupplier desiredPitch, desiredSpin;

        public ChangeState(DoubleSupplier pitchSupplier, DoubleSupplier spinSupplier) {
            addRequirements(Shooter.this);
            desiredPitch = pitchSupplier;
            desiredSpin = spinSupplier;
        }
        
        @Override
        public void execute() {
            if (desiredPitch != null) setGoalPitch(desiredPitch.getAsDouble());
            if (desiredSpin != null) setGoalSpin(desiredSpin.getAsDouble());
        }
    
        @Override
        public boolean isFinished() {
            return (desiredPitch == null || (Math.abs(getPitch() - desiredPitch.getAsDouble()) < Constants.Shooter.pitchTolerance))
                && (desiredSpin == null || (Math.abs(getSpin() - desiredSpin.getAsDouble()) < Constants.Shooter.spinTolerance));
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                setGoalPitch(Constants.Shooter.restingPitch);
                stopSpin();
            }
            super.end(interrupted);
        }
    }
}
