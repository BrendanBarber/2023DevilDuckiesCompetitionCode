package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTestSubsystem extends SubsystemBase {
    
    Talon motorController = new Talon(0);

    @Override
    public void periodic(){
        motorController.setVoltage(RobotController.getBatteryVoltage());
    }

}
