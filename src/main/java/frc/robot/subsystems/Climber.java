package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Climber extends SubsystemBase {
    private CANSparkBase mLeftController, mRightController;
    
    public Climber(){
        mLeftController = new CANSparkMax(Constants.Climber.motorLID, MotorType.kBrushless);
        mRightController = new CANSparkMax(Constants.Climber.motorRID, MotorType.kBrushless);
        mRightController.follow(mLeftController, true);
    }

    // /** @return radians */
    // private double getHeight() {
    //     return Conversions.rotationsToMeters(motorController.getEncoder().getPosition(), Constants.Climber.sprocketCircumference);
    // }
    
    // /** @param goalPitch radians */
    // private void setGoalPitch(double goalPitch) {
    //     motorController.getPIDController().setReference(Conversions.metersToRotations(goalPitch, Constants.Climber.sprocketCircumference), CANSparkBase.ControlType.kPosition);
    // }

    private void set(SpinState ss) {
        mLeftController.set(ss.multiplier * Constants.Climber.climbSpeed);
    }

    public class ChangeState extends Command {
        private SpinState desiredState;

        public ChangeState(SpinState ss) {
            desiredState = ss;
            addRequirements(Climber.this);
        }

        @Override
        public void execute() {
            set(desiredState);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) set(SpinState.ST);
            super.end(interrupted);
        }
    }
}
