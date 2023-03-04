package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class SpitOutGamePiece extends CommandBase {
    ClawSubsystem clawSubsystem;
    double startTime;
    public SpitOutGamePiece(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(this.clawSubsystem);
    }
    public void initialize() {
        SmartDashboard.putNumber("started", startTime);
        startTime = Timer.getFPGATimestamp();
    }
    public void execute() {
        double controlEffort = Preferences.getDouble(Constants.ClawCharacteristics.clawSpeedKey, this.clawSubsystem.getClawSpeed());
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, controlEffort);
    }
    @Override
    public boolean isFinished() {
        return !(this.clawSubsystem.sonarSensor.get()) || (Timer.getFPGATimestamp() - startTime) > 5.;
    }
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("ended", Timer.getFPGATimestamp());
        this.clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
    }
}
