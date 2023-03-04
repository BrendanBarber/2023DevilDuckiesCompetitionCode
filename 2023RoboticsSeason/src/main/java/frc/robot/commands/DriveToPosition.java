package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.DuckGearUtil;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPosition extends CommandBase {
        DrivetrainSubsystem drivetrainSubsystem;
        double leftPosition;
        double rightPosition;
        double avgPosition;
        double relativePositionTicks;
        double originalPitch;
        boolean startedGoingUp = false;
        public DriveToPosition(DrivetrainSubsystem drivetrain, double relativePositionMeters) {
                relativePositionTicks = DuckGearUtil.metersToEncoderTicks(
                        relativePositionMeters, 
                        Constants.DrivetrainCharacteristics.gearing, 
                        2048.0, 
                        Constants.DrivetrainCharacteristics.wheelRadiusMeters);

                drivetrainSubsystem = drivetrain;
                addRequirements(drivetrainSubsystem);
        }

        public void initialize(){
                leftPosition = drivetrainSubsystem.MainLeftMotorBack.getSelectedSensorPosition() + relativePositionTicks;
                rightPosition = drivetrainSubsystem.MainRightMotorBack.getSelectedSensorPosition() + relativePositionTicks;
                avgPosition = (leftPosition + rightPosition)/2.;
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Coast);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Coast);
                originalPitch = drivetrainSubsystem.gyro.getPitch();
                startedGoingUp = false;
        }

        public void execute() {
                SmartDashboard.putNumber("is moving position", Timer.getFPGATimestamp());
                if (drivetrainSubsystem.gyro.getPitch()-originalPitch > 11.5 && !startedGoingUp) {
                        startedGoingUp = true;
                }
                double controlEffortForward = drivetrainSubsystem.pidMovement.calculate(drivetrainSubsystem.getAvgEncoderPosition(), leftPosition);
                controlEffortForward = MathUtil.clamp(controlEffortForward, -.75, .75);
                drivetrainSubsystem.robotDrive.arcadeDrive(.5, 0.);
        }

        public boolean isFinished() {
                return  startedGoingUp;/*&& drivetrainSubsystem.gyro.getPitch()-originalPitch < 6.0;*/
        }

        public void end(boolean gotInterrupted) {
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Brake);
        }
}