package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DeflectorConstants.DeflectorState;

public class Deflector extends SubsystemBase{
    private CANSparkMax motor;

    public Deflector(){
        motor = new CANSparkMax(Constants.DeflectorConstants.deflectorMotorID, MotorType.kBrushless);
    }

    private void setGoalPos(double pos){
        motor.getPIDController().setReference(pos, CANSparkMax.ControlType.kSmartMotion);
    }

    private void stopMotor(){
        motor.stopMotor();
    }

    private double getPos(){
        return motor.getEncoder().getPosition();
    }

    public class ChangeState extends Command{
        private final Constants.DeflectorConstants.DeflectorState dState;

        public ChangeState(Constants.DeflectorConstants.DeflectorState ds){
            dState = ds;
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPos(dState.pos);
            super.initialize();
        }

        @Override
        public boolean isFinished(){
            return(Math.abs(getPos() -dState.pos) <= Constants.DeflectorConstants.deflectorTolerance);
        }

        @Override
        public void end(boolean interrupted){
            if(interrupted) setGoalPos(Constants.DeflectorConstants.DeflectorState.DOWN.pos);
            super.end(interrupted);
        }
    }
}

