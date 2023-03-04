package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DriveArm extends CommandBase {
        ArmSubsystem armSubsystem;
        double deadband;
        public DriveArm(ArmSubsystem armSubsystem) {
                this.armSubsystem = armSubsystem;
                this.deadband = Constants.OperatorConstants.movementDeadband;
                addRequirements(this.armSubsystem);
        }
        public void execute() {
                double speed = 45.0; //degrees per second
                double movementAxis = -RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.movementAxis);
                boolean holdingBumper = RobotContainer.operatorJoystick.rightBumper().getAsBoolean();

                if (holdingBumper == true) {
                        speed = 5.0;
                }
                if (movementAxis > this.deadband) {
                        this.armSubsystem.commandAngle(this.armSubsystem.getAngle() + speed * 0.05);
                } else if (movementAxis < -this.deadband) {
                        this.armSubsystem.commandAngle(this.armSubsystem.getAngle() - speed * 0.05);
                }
        }
        public boolean isFinished() {
                return false;
        }
}
