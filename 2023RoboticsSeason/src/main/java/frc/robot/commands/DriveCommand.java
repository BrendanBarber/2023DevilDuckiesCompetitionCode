package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase{

    DrivetrainSubsystem drivetrainSubsystem;
    public DriveCommand(DrivetrainSubsystem subsystem){
        drivetrainSubsystem = subsystem;
        addRequirements(drivetrainSubsystem);
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double y = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.ForwardDriveAxis);
        double x = RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.TurningDriveAxis);
        //SmartDashboard.putBoolean("is not turning", Math.abs(y) >= Constants.DrivetrainCharacteristics.deadband);
        if (Math.abs(y) >= Constants.DrivetrainCharacteristics.deadband) {
            drivetrainSubsystem.robotDrive.setMaxOutput(Constants.DrivetrainCharacteristics.speedScale);
            drivetrainSubsystem.robotDrive.arcadeDrive(y,-x);
        } else {
            drivetrainSubsystem.robotDrive.setMaxOutput(1.0);
            double controlEFfort = x * Constants.DrivetrainCharacteristics.turnSpeedScale;
            drivetrainSubsystem.robotDrive.tankDrive(controlEFfort, -controlEFfort);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}
