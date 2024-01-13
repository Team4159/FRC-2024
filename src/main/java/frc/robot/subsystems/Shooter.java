package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
    private Kinesthetics kinesthetics;
    
    private TalonFX angleMotorController;
    private MotionMagicDutyCycle angleDutyCycle;

    private CANSparkFlex shooterMotorController;
    
    public Shooter(Kinesthetics k) {
        kinesthetics = k;
        angleMotorController = new TalonFX(Constants.Shooter.angleMotorIDs[0]);
        for (int n = 1; n < Constants.Shooter.angleMotorIDs.length; n++)
            new TalonFX(Constants.Shooter.angleMotorIDs[n]).setControl(new Follower(Constants.Shooter.angleMotorIDs[0], false));
    }

    public double getPitch() {
        return angleMotorController.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    public void setGoalPitch(double goalRadians) {
        angleMotorController.setControl(angleDutyCycle.withPosition(goalRadians).withSlot(0));
    }

    public double getSpin() {
        return shooterMotorController.getEncoder().getVelocity() * Math.PI / 30;
    }

    public void setGoalSpin(double goalRadiansPerSecond) { // TODO: figure out how to get radians per sec from m/s
        shooterMotorController.getPIDController().setReference(goalRadiansPerSecond * 30/Math.PI, CANSparkMax.ControlType.kVelocity);
    }
}
