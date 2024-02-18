package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Intake extends SubsystemBase {
    private CANSparkBase angleMotorController, intakeMotorController;

    public Intake() {
        angleMotorController = new CANSparkFlex(Constants.Intake.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorController = new CANSparkMax(Constants.Intake.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
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
        return Units.rotationsPerMinuteToRadiansPerSecond(intakeMotorController.getEncoder().getVelocity());
    }

    public void setSpin(SpinState ss) {
        intakeMotorController.set(ss.multiplier * Constants.Intake.intakeSpin);
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
            return Math.abs(getPitch() - desiredState.pitch) < Constants.Intake.pitchTolerance &&
                Math.abs(getSpin() - desiredState.spin.multiplier * Constants.Intake.intakeSpin) < Constants.Intake.spinTolerance;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) setGoalPitch(Constants.Intake.IntakeState.STOW.pitch);
            setSpin(SpinState.ST);
            super.end(interrupted);
        }
    }
}