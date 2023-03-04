package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Balance extends CommandBase{

        private DrivetrainSubsystem drivetrainSubsystem;
        public Balance(DrivetrainSubsystem drivetrainSubsystem){
                this.drivetrainSubsystem = drivetrainSubsystem;
        }

        @Override
        public void initialize(){
                drivetrainSubsystem.pidGyroPitch.setSetpoint(0.0);
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Brake);
        }
        @Override
        public void execute() {
                double controlEffort = drivetrainSubsystem.pidGyroPitch.calculate(-drivetrainSubsystem.gyro.getPitch(), 0.0);
                controlEffort = MathUtil.clamp(controlEffort, -.5, .5);
                SmartDashboard.putNumber("is moving gyro", controlEffort);
                drivetrainSubsystem.robotDrive.arcadeDrive(controlEffort, 0.);
        }

        public boolean isFinished() {
                return false;
        }

        public void end(boolean gotInterrupted) {
                drivetrainSubsystem.MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.leftTopMotor.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.rightTopMotor.setNeutralMode(NeutralMode.Brake);
                drivetrainSubsystem.tankDriveVolts(0, 0);
        }
}
