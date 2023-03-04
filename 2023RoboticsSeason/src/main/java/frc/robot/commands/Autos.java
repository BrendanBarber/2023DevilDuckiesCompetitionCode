// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.DuckAutoProfile;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  private static DuckAutoProfile highTaxiPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.HIGH_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }
  private static DuckAutoProfile midTaxiPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }  
  private static DuckAutoProfile lowTaxiPath(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }


  private static DuckAutoProfile midBalance(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_BALANCE;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
      Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters), true);
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory), 
        new WaitCommand(1), 
        new InstantCommand(()->{
          drivetrainSubsystem.gyro.resetPitch();
          drivetrainSubsystem.MainLeftMotorBack.setSelectedSensorPosition(0);
          drivetrainSubsystem.MainRightMotorBack.setSelectedSensorPosition(0);
        })
        ,new BalanceComplexCommand(drivetrainSubsystem, false)
      );
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }
  private static DuckAutoProfile coneTaxi(DrivetrainSubsystem drivetrainSubsystem, ClawSubsystem clawSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.CONE_TAXI;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(new SpitOutGamePiece(clawSubsystem),
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }

  private static DuckAutoProfile justBalance(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.MID_BALANCE;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      1, 
    0.75), true);
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(new BalanceComplexCommand(drivetrainSubsystem, false), startPosition);
  }

  private static DuckAutoProfile justTouch(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.DOCK;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(
        runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();
    return new DuckAutoProfile(command, startPosition);
  }

  /*private static DuckAutoProfile forwardAuto(DrivetrainSubsystem drivetrainSubsystem){
    String pathName = Constants.AutoTrajectoryFileNames.LOW_FORWARD;
    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(
      Constants.DrivetrainCharacteristics.maxAutoVelocityMeters, 
    Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters));
    SequentialCommandGroup command = new SequentialCommandGroup(runPath(drivetrainSubsystem, pathTrajectory));
    Pose2d startPosition = pathTrajectory.getInitialPose();

    return new DuckAutoProfile(command, startPosition);
  }*/

  //auto factory
  private static CommandBase runPath(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory pathTrajectory) {
    
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    drivetrainSubsystem::getPose, 
    drivetrainSubsystem::resetOdometry, 
    drivetrainSubsystem.ramseteController, 
    drivetrainSubsystem.drivetrainKinematics, 
    new SimpleMotorFeedforward(Constants.DrivetrainCharacteristics.kS, Constants.DrivetrainCharacteristics.kV,Constants.DrivetrainCharacteristics.kA),
    drivetrainSubsystem::getWheelSpeeds, 
    new PIDConstants(Constants.DrivetrainCharacteristics.kP, 0.0, Constants.DrivetrainCharacteristics.kD),
    drivetrainSubsystem::tankDriveVolts,
    Constants.eventMap,  
    true,
    drivetrainSubsystem);
    return autoBuilder.fullAuto(pathTrajectory).andThen(() -> drivetrainSubsystem.tankDriveVolts(0,0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static void pushAutosToDashboard(SendableChooser<DuckAutoProfile> autonomousMode,
      DrivetrainSubsystem drivetrainSubsystem, ClawSubsystem clawSubsystem) {

    autonomousMode.setDefaultOption("Do nothing", new DuckAutoProfile());
    
    //autonomousMode.addOption("forward auto", forwardAuto(drivetrainSubsystem));
    autonomousMode.addOption("low taxi auto", lowTaxiPath(drivetrainSubsystem));
    autonomousMode.addOption("mid taxi auto", midTaxiPath(drivetrainSubsystem));
    autonomousMode.addOption("high taxi auto", highTaxiPath(drivetrainSubsystem));
    autonomousMode.addOption("mid balance auto", midBalance(drivetrainSubsystem));
    autonomousMode.addOption("cone taxi auto", coneTaxi(drivetrainSubsystem, clawSubsystem));
    autonomousMode.addOption("just balance", justBalance(drivetrainSubsystem));
    autonomousMode.addOption("just touch", justTouch(drivetrainSubsystem));
  }
}