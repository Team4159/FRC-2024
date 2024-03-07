package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
        goalPitch = MathUtil.clamp(goalPitch, 0, Constants.Shooter.maximumPitch);
        angleMotorController.set(
            Constants.Shooter.shooterPID.calculate(getPitch(), goalPitch)
            + Constants.Shooter.kF * Math.cos(getPitch())
            + Constants.Shooter.pitchOffset
        );
        //angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch) + Constants.Shooter.pitchOffset, CANSparkBase.ControlType.kSmartMotion);
    }

    /** @return radians / second */
    public double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

    /** @param goalSpin radians / second */
    private void setGoalSpin(double goalSpin) {
        shooterMLeftController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalSpin), CANSparkBase.ControlType.kSmartVelocity); 
    }

    /** @param goalNoteVel meters / second */
    private double velocityToSpin(double goalNoteVel) {
        return Constants.Shooter.shooterFeedForward.calculate(goalNoteVel);
    }

    public void stopSpin() {
        shooterMLeftController.stopMotor();
    }

    private void setNeck(SpinState ss) {
        neckMotorController.set(ss.multiplier * Constants.Shooter.neckSpeed);
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> new Pair<>(pitch, null), false);
    }

    public ChangeState toSpin(double spin) {
        return new ChangeState(() -> new Pair<>(null, spin), false);
    }

    public class ChangeState extends Command {
        private boolean continuous = false;
        /** @param desiredState Pair< radians , meters / second > */
        private ShooterStateSupplier desiredState;
        
        public ChangeState(ShooterStateSupplier shooterStateSupplier) {
            this(shooterStateSupplier, false);
        }

        public ChangeState(ShooterStateSupplier shooterStateSupplier, boolean continuous) {
            desiredState = shooterStateSupplier;
            this.continuous = continuous;
            addRequirements(Shooter.this);
        }
        
        @Override
        public void execute() {
            var state = desiredState.get();
            if (state.getFirst() != null) setGoalPitch(state.getFirst());
            if (state.getSecond() != null) setGoalSpin(velocityToSpin(state.getSecond()));
        }
    
        @Override
        public boolean isFinished() {
            if (continuous) return false;
            var state = desiredState.get();
            return (state.getFirst() == null || (Math.abs(getPitch() - state.getFirst()) < Constants.Shooter.pitchTolerance))
                && (state.getSecond() == null || (Math.abs(getSpin() - velocityToSpin(state.getSecond())) < Constants.Shooter.spinTolerance));
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted && !continuous) {
                setGoalPitch(0);
                stopSpin();
            }
            super.end(interrupted);
        }

        @FunctionalInterface
        public interface ShooterStateSupplier {
            public Pair<Double, Double> get();
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
