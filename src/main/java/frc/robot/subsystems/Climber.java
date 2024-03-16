package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private CANSparkBase motorController;
    
    public Climber(){
        motorController = new CANSparkFlex(Constants.Climber.motorID, MotorType.kBrushless);
    }

    /** @return radians */
    private double getHeight() {
        return Conversions.rotationsToMeters(motorController.getEncoder().getPosition(), Constants.Climber.sprocketCircumference);
    }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        motorController.getPIDController().setReference(Conversions.metersToRotations(goalPitch, Constants.Climber.sprocketCircumference), CANSparkBase.ControlType.kPosition);
    }

    public class Raise extends Command {
        public Raise() {
            addRequirements(Climber.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(Constants.Climber.maximumHeight);
        }

        @Override
        public boolean isFinished() {
            return MathUtil.isNear(Constants.Climber.maximumHeight, getHeight(), Constants.Climber.heightTolerance);
        }

        @Override
        public void end(boolean interrupted) {
            setGoalPitch(Constants.Climber.hookHeight);
            super.end(interrupted);
        }
    }
}
