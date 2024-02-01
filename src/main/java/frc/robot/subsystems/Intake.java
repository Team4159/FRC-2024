package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Intake extends SubsystemBase {
    private CANSparkFlex angleMotorController;
    private CANSparkMax intakeMotorController;

    public Intake() {
        angleMotorController = new CANSparkFlex(Constants.Intake.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorController = new CANSparkMax(Constants.Intake.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }
    
    public double getPitch() {
        return angleMotorController.getEncoder().getPosition();
    }
    
    public void setGoalPitch(double position) {
        angleMotorController.getPIDController().setReference(position, CANSparkBase.ControlType.kDutyCycle);
    }

    public double getSpin() {
        return intakeMotorController.getEncoder().getVelocity();
    }

    public void setSpin(SpinState ss) {
        intakeMotorController.set(ss.multiplier * Constants.Intake.intakeSpeed);
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
                Math.abs(getSpin() - desiredState.spin.multiplier * Constants.Intake.intakeSpeed) < Constants.Intake.spinTolerance;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) setGoalPitch(Constants.Intake.IntakeState.STOW.pitch);
            setSpin(SpinState.ST);
            super.end(interrupted);
        }
    }
}