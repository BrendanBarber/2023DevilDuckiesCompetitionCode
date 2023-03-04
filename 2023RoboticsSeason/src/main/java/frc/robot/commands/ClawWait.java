package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawWait extends CommandBase{

        ClawSubsystem clawSubsystem;
        double time;
        double startTime;

        public ClawWait(ClawSubsystem clawSubsystem){
                this.clawSubsystem = clawSubsystem;
        }

        @Override
        public void initialize(){
                time = clawSubsystem.getClawTime();
                startTime = Timer.getFPGATimestamp();
        }

        public boolean isFinished(){
                return (Timer.getFPGATimestamp() - startTime) > time;
        }
        
}
