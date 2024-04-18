package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Intake extends SubsystemBase {
    private CANSparkBase angleMotorController, intakeMotorController, feederMotorController;

    public Intake() {
        angleMotorController = new CANSparkFlex(Constants.Intake.angleMotorID, MotorType.kBrushless);
        intakeMotorController = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotorController.setInverted(true);
        feederMotorController = new CANSparkMax(Constants.Intake.feederMotorID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        var realPitch = MathUtil.angleModulus(getPitch());
        if (realPitch < 0) realPitch += 2 * Math.PI;
        assert(Math.abs(realPitch - desiredPitch) < Math.PI);
        angleMotorController.set(
            Constants.Intake.anglePID.calculate(realPitch, desiredPitch)
            + Constants.Intake.kG * Math.cos(realPitch)
        );
    }

    /** @return radians */
    private double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }
    
    private double desiredPitch;

    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        desiredPitch = MathUtil.clamp(goalPitch, Constants.Intake.IntakeState.STOW.pitch, Constants.Intake.IntakeState.DOWN.pitch);
        // angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kSmartMotion);
    }

    /** @return radians / second */
    private double getSpin() {
        return Units.rotationsPerMinuteToRadiansPerSecond(intakeMotorController.getEncoder().getVelocity());
    }

    private void setSpin(SpinState ss) {
        intakeMotorController.set(ss.multiplier * Constants.Intake.intakeSpin);
        feederMotorController.set(ss.multiplier * Constants.Intake.feederSpin);
    }

    public class ChangeState extends Command {
        private final Constants.Intake.IntakeState desiredState;

        public ChangeState(Constants.Intake.IntakeState ds) {
            desiredState = ds;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            setGoalPitch(desiredState.pitch);
            setSpin(desiredState.spin);
            super.initialize();
        }

        @Override
        public boolean isFinished() {
            return MathUtil.isNear(desiredState.pitch, getPitch(), Constants.Intake.pitchTolerance) &&
                MathUtil.isNear(desiredState.spin.multiplier * Constants.Intake.intakeSpin, getSpin(), Constants.Intake.spinTolerance);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                setGoalPitch(Constants.Intake.IntakeState.STOW.pitch);
                setSpin(Constants.Intake.IntakeState.STOW.spin);
            }
            super.end(interrupted);
        }
    }
}