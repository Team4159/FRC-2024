package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflector extends SubsystemBase{
    private CANSparkMax angleMotorControllerL;//, angleMotorControllerR;

    public Deflector(){
        angleMotorControllerL = new CANSparkMax(Constants.Deflector.lMotorID, MotorType.kBrushless);
        //angleMotorControllerR = new CANSparkMax(Constants.Deflector.rMotorID, MotorType.kBrushless);
        //angleMotorControllerR.follow(angleMotorControllerL, true);
        //angleMotorControllerR.setInverted(true);
    }

    /** @return radians */
    public double getLeftPitch() {
        return Units.rotationsToRadians(angleMotorControllerL.getEncoder().getPosition());
    }

    // public double getRightPitch() {
    //     return Units.rotationsToRadians(angleMotorControllerR.getEncoder().getPosition());
    // }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        angleMotorControllerL.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kPosition);
        //angleMotorControllerR.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kPosition);
        //angleMotorControllerR.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kPosition);
    }

    public class Raise extends Command {
        private double desiredPitch;
        public Raise() {
            desiredPitch = Constants.Deflector.maximumPitch;
            addRequirements(Deflector.this);
        }

        public Raise(double pitch) {
            desiredPitch = pitch;
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(desiredPitch);
            super.initialize();
        }

        @Override
        public boolean isFinished() {
            //return Math.abs(desiredPitch - getRightPitch()) < Constants.Deflector.pitchTolerance;
            //&& Math.abs(desiredPitch - getLeftPitch()) < Constants.Deflector.pitchTolerance;
            return false;
        }

        @Override
        public void end(boolean interrupted){
            setGoalPitch(0);
            super.end(interrupted);
        }
    }
}