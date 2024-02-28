package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflector extends SubsystemBase{
    private CANSparkMax angleMotorController;

    public Deflector(){
        angleMotorController = new CANSparkMax(Constants.DeflectorConstants.angleMotorID, MotorType.kBrushless);
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kSmartMotion);
    }

    public class ChangeState extends Command{
        private final Constants.DeflectorConstants.DeflectorState desiredState;

        public ChangeState(Constants.DeflectorConstants.DeflectorState ds){
            desiredState = ds;
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(desiredState.pitch);
            super.initialize();
        }

        @Override
        public boolean isFinished(){
            return(Math.abs(getPitch() - desiredState.pitch) <= Constants.DeflectorConstants.deflectorTolerance);
        }

        @Override
        public void end(boolean interrupted){
            if(interrupted) setGoalPitch(Constants.DeflectorConstants.DeflectorState.DOWN.pitch);
            super.end(interrupted);
        }
    }
}

