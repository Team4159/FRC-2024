package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
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
        shooterMRightController.setInverted(true);
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    /** @return radians */
    private double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() - Constants.Shooter.pitchOffset);
    }

    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        goalPitch = MathUtil.clamp(goalPitch, Constants.Shooter.minimumPitch, Constants.Shooter.maximumPitch);
        angleMotorController.set(
            Constants.Shooter.shooterPID.calculate(getPitch(), goalPitch)
            + Constants.Shooter.kF * Math.cos(getPitch())
            + Constants.Shooter.pitchOffset
        );
        //angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch) + Constants.Shooter.pitchOffset, CANSparkBase.ControlType.kSmartMotion);
    }

    /** @return radians / second */
    private double getLSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

    /** @return radians / second */
    private double getRSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMRightController.getEncoder().getVelocity());
    }

    /**
     * @param goalLSpin radians / second
     * @param goalRSpin radians / second 
     * */
    private void setGoalSpin(double goalLSpin, double goalRSpin) {
        shooterMLeftController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalLSpin), CANSparkBase.ControlType.kSmartVelocity);
        shooterMRightController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalRSpin), CANSparkBase.ControlType.kSmartVelocity);  
    }

    private void setNeck(SpinState ss) {
        neckMotorController.set(ss.multiplier * Constants.Shooter.neckSpeed);
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> new ShooterCommand(pitch, null, null), false);
    }

    public ChangeState toSpin(double lSpin, double rSpin) {
        return new ChangeState(() -> new ShooterCommand(null, lSpin, rSpin), false);
    }

    public ChangeState stopShooter() {
        return new ChangeState(() -> new ShooterCommand(Constants.Shooter.minimumPitch, 0d), false);
    }

    /**
     * @param pitch radians
     * @param lSpin radians / second
     * @param rSpin radians / second
     * */
    public static record ShooterCommand(Double pitch, Double lSpin, Double rSpin) {
        public ShooterCommand(Double pitch, Double spin) {
            this(pitch, spin, spin);
        }

        public boolean hasSpin() {
            return lSpin != null || rSpin != null;
        }
    }

    public class ChangeState extends Command {
        private boolean continuous = false;
        private ShooterStateSupplier desiredState;

        public ChangeState(ShooterStateSupplier shooterStateSupplier, boolean continuous) {
            desiredState = shooterStateSupplier;
            this.continuous = continuous;
            addRequirements(Shooter.this);
        }
        
        @Override
        public void execute() {
            var state = desiredState.get();
            if (state.pitch() != null) setGoalPitch(state.pitch());
            if (state.hasSpin()) setGoalSpin(state.lSpin(), state.rSpin());
        }
    
        @Override
        public boolean isFinished() {
            if (continuous) return false;
            var state = desiredState.get();
            return MathUtil.isNear(state.pitch(), getPitch(), Constants.Shooter.pitchTolerance)
                && MathUtil.isNear(state.lSpin(), getLSpin(), Constants.Shooter.spinTolerance)
                && MathUtil.isNear(state.rSpin(), getRSpin(), Constants.Shooter.spinTolerance);
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted && !continuous) {
                setGoalPitch(Constants.Shooter.minimumPitch);
                shooterMLeftController.stopMotor();
                shooterMRightController.stopMotor();
            }
            super.end(interrupted);
        }

        @FunctionalInterface
        public interface ShooterStateSupplier{
            public ShooterCommand get();
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
        public boolean isFinished() {
            return !beamBreakMode || !kinesthetics.shooterHasNote();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) setNeck(SpinState.ST);
            super.end(interrupted);
        }
    }
}
