package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
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
        angleMotorController = new CANSparkFlex(Constants.Shooter.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.follow(shooterMLeftController, true); // for now, no spin.
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() - Constants.Shooter.pitchOffset);
    }

    /** @param goalPitch radians */
    public void setGoalPitch(double goalPitch) {
        angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch) + Constants.Shooter.pitchOffset, CANSparkBase.ControlType.kSmartMotion);
    }

    /** @return radians / second */
    public double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

    /** @param goalNoteVel meters / second */
    public void setGoalSpin(double goalNoteVel) {
        shooterMLeftController.getPIDController().setReference(
            Conversions.RadiansPSToRPM(Constants.Shooter.shooterFeedForward.calculate(goalNoteVel)),
            // Conversions.MPSToRPS(goalNoteVel, Units.inchesToMeters(4) * Math.PI) * 60,
            CANSparkBase.ControlType.kSmartVelocity
        );
    }

    public void stopSpin() {
        shooterMLeftController.stopMotor();
    }

    private void setNeck(SpinState ss) {
        neckMotorController.set(ss.multiplier * Constants.Shooter.neckSpeed);
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> pitch, null);
    }

    public ChangeState toSpin(double spin) {
        return new ChangeState(null, () -> spin);
    }

    public class ChangeState extends Command {
        private boolean continuous = false;
        private DoubleSupplier desiredPitch, desiredSpin;

        public ChangeState(DoubleSupplier pitchSupplier, DoubleSupplier spinSupplier) {
            desiredPitch = pitchSupplier;
            desiredSpin = spinSupplier;
            addRequirements(Shooter.this);
        }

        public ChangeState(DoubleSupplier pitchSupplier, DoubleSupplier spinSupplier, boolean continuous) {
            desiredPitch = pitchSupplier;
            desiredSpin = spinSupplier;
            this.continuous = continuous;
            addRequirements(Shooter.this);
        }
        
        @Override
        public void execute() {
            if (desiredPitch != null) setGoalPitch(desiredPitch.getAsDouble());
            if (desiredSpin != null) setGoalSpin(desiredSpin.getAsDouble());
        }
    
        @Override
        public boolean isFinished() {
            if (continuous) return false;
            return (desiredPitch == null || (Math.abs(getPitch() - desiredPitch.getAsDouble()) < Constants.Shooter.pitchTolerance))
                && (desiredSpin == null || (Math.abs(getSpin() - desiredSpin.getAsDouble()) < Constants.Shooter.spinTolerance));
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted && !continuous) {
                setGoalPitch(0);
                stopSpin();
            }
            super.end(interrupted);
        }
    }

    public class ChangeNeck extends Command {
        private Kinesthetics kinesthetics;

        private boolean beamBreakMode; // should this command end when the beam break opens
        private SpinState desiredNeck;

        public ChangeNeck(SpinState ss) {
            desiredNeck = ss;
            addRequirements(Shooter.this);
        }

        public ChangeNeck(Kinesthetics k, SpinState ss) {
            kinesthetics = k;
            desiredNeck = ss;
            beamBreakMode = kinesthetics.shooterHasNote();
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize() {
            setNeck(desiredNeck);
        }

        @Override
        public boolean isFinished() { // this needs to be changed if it runs before initialize does
            return !beamBreakMode || !kinesthetics.shooterHasNote();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) setNeck(SpinState.ST);
            super.end(interrupted);
        }
    }
}
