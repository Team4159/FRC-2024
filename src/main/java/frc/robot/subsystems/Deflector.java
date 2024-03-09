package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflector extends SubsystemBase{
    private CANSparkMax angleMotorControllerL, angleMotorControllerR;

    public Deflector(){
        angleMotorControllerL = new CANSparkMax(Constants.Deflector.lMotorID, MotorType.kBrushless);
        angleMotorControllerR = new CANSparkMax(Constants.Deflector.rMotorID, MotorType.kBrushless);
        angleMotorControllerR.follow(angleMotorControllerL);
    }

    /** @return radians */
    public double getPitch() {
        return Units.rotationsToRadians(angleMotorControllerL.getEncoder().getPosition());
    }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        angleMotorControllerL.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kSmartMotion);
    }

    public class Raise extends Command {
        public Raise() {
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(Constants.Deflector.maximumPitch);
            super.initialize();
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted){
            setGoalPitch(0);
            super.end(interrupted);
        }
    }
}