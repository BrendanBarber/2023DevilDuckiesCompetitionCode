package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class GrabGamePiece extends CommandBase {
    ClawSubsystem clawSubsystem;
    double startTime;
    public GrabGamePiece(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(this.clawSubsystem);
    }
    public void initialize() {
        this.clawSubsystem.isClosing = true;
        startTime = Timer.getFPGATimestamp();
    }
    public void execute() {
        double controlEffort = Preferences.getDouble(Constants.ClawCharacteristics.clawSpeedKey, this.clawSubsystem.getClawSpeed());
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, -controlEffort);
    }
    public boolean isFinished(boolean interrupted) {
        return this.clawSubsystem.getDisabledState();
    }
}
