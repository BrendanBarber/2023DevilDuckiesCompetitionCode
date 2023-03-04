package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ForwardForever extends CommandBase {
        DrivetrainSubsystem drivetrain;

        public ForwardForever(DrivetrainSubsystem drivetrainSubsystem) {
                this.drivetrain = drivetrainSubsystem;
                addRequirements(this.drivetrain);
        }

        public void execute() {
                // kS xor kV must be 0
                double voltage = Constants.DrivetrainCharacteristics.kS + Constants.DrivetrainCharacteristics.kV;
                drivetrain.tankDriveVolts(voltage, voltage);
        }
}
