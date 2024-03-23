package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflector extends SubsystemBase{
    private CANSparkMax angleMotorController;

    public Deflector() {
        angleMotorController = new CANSparkMax(Constants.Deflector.motorID, MotorType.kBrushless);
    }

    // /** @return radians */
    // private double getPitch() {
    //     return Units.rotationsToRadians(angleMotorControllerL.getEncoder().getPosition());
    // }
    
    /** @param goalPitch radians */
    private void setGoalPitch(double goalPitch) {
        angleMotorController.getPIDController().setReference(Units.radiansToRotations(goalPitch), CANSparkBase.ControlType.kPosition);
    }

    public class Raise extends Command {
        public Raise() {
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(Constants.Deflector.maximumPitch);
        }
    }

    public class Lower extends Command {
        public Lower() {
            addRequirements(Deflector.this);
        }

        @Override
        public void initialize(){
            setGoalPitch(0);
        }
    }
}