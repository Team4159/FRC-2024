package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;

public class Intake extends SubsystemBase {
   private CANSparkFlex angleMotorController;
   private CANSparkMax intakeMotorController;

   private IntakeState intakeState;

    public Intake() {
        angleMotorController = new CANSparkFlex(Constants.Intake.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorController = new CANSparkMax(Constants.Intake.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);

        intakeState = IntakeState.INITIAL;
    }

    public void setDesiredState(IntakeState intakeState) {
        this.intakeState = intakeState;
        if (intakeState.equals(IntakeState.OFF)) angleMotorController.set(0);
        setIntakePitch(intakeState.setpoint);
    }

    public IntakeState getState() {
        return intakeState;
    }
    
    public void setIntakePitch(double position) {
        // TODO consider using MotionMagic AngleDutyCycle? this is a spark flex so we can't use the ctre one
        angleMotorController.getPIDController().setReference(position, CANSparkBase.ControlType.kPosition);
    }

    public void setIntakeSpin(boolean reversed) {
        intakeMotorController.set(reversed ? Constants.Intake.intakeSpeed : -Constants.Intake.intakeSpeed);
    }

    public double getIntakePitch() {
        return angleMotorController.getEncoder().getPosition();
    }

    public double getIntakeSpin() {
        return intakeMotorController.getEncoder().getVelocity();
    }
}