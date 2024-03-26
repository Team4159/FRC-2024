package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Climber extends SubsystemBase {
    private CANSparkBase motorController;
    
    public Climber(){
        motorController = new CANSparkFlex(Constants.Climber.motorID, MotorType.kBrushless);
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
        motorController.set(ss.multiplier * Constants.Climber.climbSpeed);
    }

    public class ChangeState extends Command {
        private SpinState desiredState;

        public ChangeState(SpinState ss) {
            desiredState = ss;
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            set(desiredState);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) set(SpinState.ST);
            super.end(interrupted);
        }
    }
}
