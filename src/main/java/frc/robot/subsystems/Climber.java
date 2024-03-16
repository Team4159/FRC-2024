package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private CANSparkBase motor1;
    
    public Climber(){
        motor1 = new CANSparkFlex(Constants.Climber.motorID, MotorType.kBrushless);
    }

    public void setMotor(double percentage){
        motor1.set(percentage);
    }
}
