// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.BalanceComplexCommand;
import frc.robot.commands.ClawWait;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SpinClawBackwards;
import frc.robot.commands.SpinClawForward;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(this);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  
  public SendableChooser<DuckAutoProfile> autonomousMode = new SendableChooser<DuckAutoProfile>();

  public final static CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriverConstants.port);
  public final static CommandXboxController operatorJoystick =
      new CommandXboxController(OperatorConstants.port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Autos.pushAutosToDashboard(autonomousMode, drivetrainSubsystem, clawSubsystem);
    SmartDashboard.putData(autonomousMode);
    // Configure the trigger bindings
    configureBindings();
    //configure the auton markers
    configureMarkers();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        driverJoystick.R1().whileTrue(new Balance(drivetrainSubsystem));

        operatorJoystick.x().onTrue(new SequentialCommandGroup(new SpinClawBackwards(clawSubsystem), new ClawWait(clawSubsystem), 
        new InstantCommand( () -> {
                clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
        })));
        operatorJoystick.a().onTrue(new SequentialCommandGroup(new SpinClawForward(clawSubsystem), new ClawWait(clawSubsystem), 
                new InstantCommand( () -> {
                        clawSubsystem.motor.set(ControlMode.PercentOutput, 0.0);
                })));
        operatorJoystick.y().onTrue(new InstantCommand(()->{
          armSubsystem.resetAngle(45.0);
        }));
        operatorJoystick.b().onTrue(new InstantCommand(()->{
          armSubsystem.resetAngle(90.0);
        }));

        driverJoystick.square().whileTrue(new RotateToAngle(drivetrainSubsystem, 180));
  }
  private void configureMarkers() {
    Constants.eventMap.put("marker1", new PrintCommand("passed marker 1"));
    Constants.eventMap.put("StartBalance", new BalanceComplexCommand(drivetrainSubsystem, false));
  }

  public DuckAutoProfile getAutonomousProfile() {
    // An example command will be run in autonomous
    return autonomousMode.getSelected();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }

  public ArmSubsystem getArmSubsystem(){
        return armSubsystem;
      }

public ClawSubsystem getClawSubsystem() {
        return clawSubsystem;
}
}
