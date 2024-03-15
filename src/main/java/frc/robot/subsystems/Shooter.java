package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Shooter extends SubsystemBase {  
    private CANSparkBase angleMotorController, shooterMLeftController, shooterMRightController, neckMotorController;

    private final MechanismLigament2d mechanism, mechanismGoal;

    public Shooter() {
        angleMotorController = new CANSparkFlex(Constants.Shooter.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.setInverted(true);
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
        
        var canvas = new Mechanism2d(400, 400);
        mechanism = new MechanismLigament2d("Shooter", 100, Units.radiansToDegrees(getPitch()), 10, new Color8Bit(255, 64, 64));
        mechanismGoal = new MechanismLigament2d("Shooter Goal", 100, Units.radiansToDegrees(getPitch()), 5, new Color8Bit(10, 10, 60));
        canvas.getRoot("A-Frame", 200, 200).append(mechanism);
        canvas.getRoot("A-Frame Goal", 200, 200).append(mechanismGoal);
        Shuffleboard.getTab("Kinesthetics").add("Shooter", canvas);
    }

    @Override
    public void periodic() {
        var d = Units.radiansToDegrees(getPitch());
        mechanism.setAngle(d);
        SmartDashboard.putNumber("d", d);
    }

    /** @return radians */
    private double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() - Constants.Shooter.pitchOffset);
    }

    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        goalPitch = MathUtil.clamp(MathUtil.angleModulus(goalPitch), Constants.Shooter.minimumPitch, Constants.Shooter.maximumPitch);
        mechanismGoal.setAngle(Units.radiansToDegrees(goalPitch));
        angleMotorController.set(
            // Constants.Shooter.shooterAngleFF.calculate()
            Constants.Shooter.shooterPID.calculate(getPitch(), goalPitch + Units.rotationsToRadians(Constants.Shooter.pitchOffset))
            + Constants.Shooter.kF * Math.cos(getPitch())
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
        setNeck(ss, 1);
    }

    private void setNeck(SpinState ss, double multiplier) {
        neckMotorController.set(ss.multiplier * multiplier * Constants.Shooter.neckSpeed);
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> new ShooterCommand(pitch, null, null), false);
    }

    public ChangeState toSpin(double lSpin, double rSpin) {
        return new ChangeState(() -> new ShooterCommand(null, lSpin, rSpin), false);
    }

    public ChangeState stopShooter() {
        return new ChangeState(() -> Constants.Shooter.idleCommand, false);
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
            return
                (state.pitch() == null || MathUtil.isNear(state.pitch(), getPitch(), Constants.Shooter.pitchTolerance)) &&
                (!state.hasSpin() || (
                    MathUtil.isNear(state.lSpin(), getLSpin(), Constants.Shooter.spinTolerance) &&
                    MathUtil.isNear(state.rSpin(), getRSpin(), Constants.Shooter.spinTolerance)
                ));
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted && !continuous) {
                setGoalPitch(Constants.Shooter.idleCommand.pitch());
                setGoalSpin(Constants.Shooter.idleCommand.lSpin(), Constants.Shooter.idleCommand.rSpin());
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
        private boolean desiredSlow;

        public ChangeNeck(SpinState ss) {
            this(ss, false);
        }

        public ChangeNeck(SpinState ss, boolean slow) {
            desiredNeck = ss;
            desiredSlow = slow;
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
            setNeck(desiredNeck, desiredSlow ? 0.5 : 1);
        }

        @Override
        public boolean isFinished() {
            return !beamBreakMode || !kinesthetics.shooterHasNote();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted || beamBreakMode) setNeck(SpinState.ST);
            super.end(interrupted);
        }
    }
}
