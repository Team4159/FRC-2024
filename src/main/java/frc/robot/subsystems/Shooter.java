package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;
import frc.robot.commands.ShooterCommand;

public class Shooter extends SubsystemBase {  
    private CANSparkBase angleMotorController, shooterMLeftController, shooterMRightController, neckMotorController;
    //for testing
    private double tPitch;

    public Shooter() {
        angleMotorController = new CANSparkFlex(Constants.Shooter.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMLeftController = new CANSparkFlex(Constants.Shooter.shooterMLeftID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController= new CANSparkFlex(Constants.Shooter.shooterMRightID,CANSparkLowLevel.MotorType.kBrushless);
        shooterMRightController.setInverted(true);
        neckMotorController = new CANSparkMax(Constants.Shooter.neckMotorID, CANSparkLowLevel.MotorType.kBrushless);
        // for testing
        tPitch = 0;
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() - Constants.Shooter.pitchOffset);
    }

    /** @param goalPitch radians */
    public void setGoalPitch(double goalPitch) {
        goalPitch = MathUtil.clamp(goalPitch, Constants.Shooter.minimumPitch, Constants.Shooter.maximumPitch);
        angleMotorController.set(
            Constants.Shooter.shooterPID.calculate(getPitch(), goalPitch)
            + Constants.Shooter.kF * Math.cos(getPitch())
            + Constants.Shooter.pitchOffset
        );
        tPitch = Units.radiansToDegrees(goalPitch);
        //angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch) + Constants.Shooter.pitchOffset, CANSparkBase.ControlType.kSmartMotion);
    }

    public void setNeckPercentage(SpinState ss, double desiredPercentage) {
        neckMotorController.set(desiredPercentage * ss.multiplier);
    }

    /** @return radians / second */
    public double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

    /** @return radians / second */
    public double getSpin1(){
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMLeftController.getEncoder().getVelocity());
    }

        /** @return radians / second */
    public double getSpin2(){
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMRightController.getEncoder().getVelocity());
    }

    /** @param goalSpin radians / second */
    public void setGoalSpin(double goalSpin) {
        shooterMLeftController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalSpin), CANSparkBase.ControlType.kSmartVelocity); 
        shooterMRightController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalSpin), CANSparkBase.ControlType.kSmartVelocity);
    }

    /** @param goalSpin1 radians / second  @param goalSpin2 radians / second*/
    public void setGoalSpin(double goalSpin1, double goalSpin2) {
        shooterMLeftController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalSpin1), CANSparkBase.ControlType.kSmartVelocity);
        shooterMRightController.getPIDController().setReference(Conversions.RadiansPSToRPM(goalSpin2), CANSparkBase.ControlType.kSmartVelocity);  
    }

    /** @param goalNoteVel meters / second */
    // private double velocityToSpin(double goalNoteVel) {
    //     return Constants.Shooter.shooterFeedForward.calculate(goalNoteVel);
    // }

    public void stopSpin() {
        shooterMLeftController.stopMotor();
    }

    private void setNeck(SpinState ss) {
        neckMotorController.set(ss.multiplier * Constants.Shooter.neckSpeed);
    }

    public void setNeck(SpinState ss, double speed) {
        neckMotorController.set(ss.multiplier * speed);
    }

    public ChangeState toPitch(double pitch) {
        return new ChangeState(() -> new ShooterCommand(pitch, getSpin1(), getSpin2()), false);
    }

    public ChangeState toSpin(double spin) {
        return new ChangeState(() -> new ShooterCommand(getPitch(), spin, spin), false);
    }

    public ChangeState toSpin(double spin1, double spin2) {
        return new ChangeState(() -> new ShooterCommand(getPitch(), spin1, spin2), false);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("target pitch", Units.degreesToRadians(tPitch));
        SmartDashboard.putNumber("current pitch", getPitch());
    }

    public class ChangeState extends Command {
        private boolean continuous = false;
        private ShooterStateSupplier desiredState;

        /** @param desiredState ShooterCommand(radians , radians / second)*/
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
            setGoalPitch(state.pitch());
            setGoalSpin(state.speed1(), state.speed2());
        }
    
        @Override
        public boolean isFinished() {
            if (continuous) return false;
            var state = desiredState.get();
            return (Math.abs(getPitch() - state.pitch()) < Constants.Shooter.pitchTolerance)
                && (Math.abs(getSpin1() - state.speed1()) < Constants.Shooter.spinTolerance)
                && (Math.abs(getSpin2() - state.speed2()) < Constants.Shooter.spinTolerance);
        }
    
        @Override
        public void end(boolean interrupted) {
            if (interrupted && !continuous) {
                setGoalPitch(0);
                stopSpin();
            }
            super.end(interrupted);
        }

        //@FunctionalInterface
        public interface ShooterStateSupplier{
            //public Triple<Double, Double, Double> get();
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
