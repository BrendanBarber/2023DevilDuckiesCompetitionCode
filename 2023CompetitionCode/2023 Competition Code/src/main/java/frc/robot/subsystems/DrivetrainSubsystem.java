package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DuckAHRS;
import frc.robot.DuckGearUtil;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;

public class DrivetrainSubsystem extends SubsystemBase{
    
  public final DifferentialDrive robotDrive;
  private final DifferentialDrivetrainSim robotDriveSim;

  // CAN devices
  public final WPI_TalonFX MainLeftMotorBack = new WPI_TalonFX(Constants.CAN.Drivetrain.BL);
  public final  WPI_TalonFX MainRightMotorBack = new WPI_TalonFX(Constants.CAN.Drivetrain.BR);
  public final WPI_TalonFX MainLeftMotorFront = new WPI_TalonFX(Constants.CAN.Drivetrain.FL);
  public final WPI_TalonFX MainRightMotorFront = new WPI_TalonFX(Constants.CAN.Drivetrain.FR);
  public final WPI_TalonFX leftTopMotor = new WPI_TalonFX(Constants.CAN.Drivetrain.TL);
  public final WPI_TalonFX rightTopMotor = new WPI_TalonFX(Constants.CAN.Drivetrain.TR);
  
  private final TalonFXSimCollection leftMotorSim;
  private final TalonFXSimCollection rightMotorSim;
  
  public DuckAHRS gyro = new DuckAHRS();

  public final RamseteController ramseteController = new RamseteController();
  public final DifferentialDriveKinematics drivetrainKinematics = new DifferentialDriveKinematics(Constants.DrivetrainCharacteristics.trackWidthMeters);

  private final DifferentialDrivePoseEstimator odometry;

  public static final Field2d field = new Field2d();
  private double gyroPitchkP = 0.037;
  private double forwardMovementkP = 0.00005;

  //proportional controllers only gang uwu
  public PIDController pidGyroPitch = new PIDController(gyroPitchkP, 0.0, 0.0);
  public PIDController pidMovement = new PIDController(forwardMovementkP, 0.0, 0.0);

  public DrivetrainSubsystem() {
    //constructor gets ran at robotInit()
    this.setDefaultCommand(new DriveCommand(this));
    
    // Reset settings
    MainLeftMotorBack.configFactoryDefault();
    MainRightMotorBack.configFactoryDefault();
    MainLeftMotorFront.configFactoryDefault();
    MainRightMotorFront.configFactoryDefault();
    leftTopMotor.configFactoryDefault();
    rightTopMotor.configFactoryDefault();

    // Setup the integrated sensor
    MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    MainLeftMotorBack.config_kP(0, 1.3);
    MainRightMotorBack.config_kP(0, 1.3);

    MainLeftMotorBack.config_kF(0, 0.0);
    MainRightMotorBack.config_kF(0, 0.0);


    // Slave the front motors to their respective back motors
    MainLeftMotorFront.follow(MainLeftMotorBack);
    MainRightMotorFront.follow(MainRightMotorBack);
    leftTopMotor.follow(MainLeftMotorBack);
    rightTopMotor.follow(MainRightMotorBack);

    // Disable voltage compensation, it's bad to be compensating voltage for a
    // system which draws loads of amps

    MainLeftMotorBack.enableVoltageCompensation(false);
    MainLeftMotorFront.enableVoltageCompensation(false);
    MainRightMotorBack.enableVoltageCompensation(false);
    MainRightMotorFront.enableVoltageCompensation(false);
    leftTopMotor.enableVoltageCompensation(false);
    rightTopMotor.enableVoltageCompensation(false);

    //Apply brake mode in auton
    MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
    MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
    MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
    MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
    leftTopMotor.setNeutralMode(NeutralMode.Brake);
    rightTopMotor.setNeutralMode(NeutralMode.Brake);

    // Invert one of the sides
    MainLeftMotorBack.setInverted(true);
    MainRightMotorBack.setInverted(false);
    MainLeftMotorFront.setInverted(InvertType.FollowMaster);
    MainRightMotorFront.setInverted(InvertType.FollowMaster);
    leftTopMotor.setInverted(InvertType.FollowMaster);
    rightTopMotor.setInverted(InvertType.FollowMaster);

    robotDrive = new DifferentialDrive(MainLeftMotorBack, MainRightMotorBack);
    robotDriveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),
        Constants.DrivetrainCharacteristics.gearing,
        2.1, // made up number
        100, // made up number
        Constants.DrivetrainCharacteristics.wheelRadiusMeters,
        Constants.DrivetrainCharacteristics.trackWidthMeters,
        null
    );
    SmartDashboard.putData("Field", field);
    
    odometry = new DifferentialDrivePoseEstimator(
        drivetrainKinematics, 
        Rotation2d.fromDegrees(-gyro.getAngle()), 
        0,0,
        new Pose2d()); //will add vision measurements once auton starts;
    leftMotorSim = MainLeftMotorBack.getSimCollection();
    rightMotorSim = MainRightMotorBack.getSimCollection();
    robotDrive.setDeadband(Constants.DrivetrainCharacteristics.deadband);
  }

  @Override
  public void periodic() {
    double leftDistanceMeters = DuckGearUtil.encoderTicksToMeters(MainLeftMotorBack.getSelectedSensorPosition(),
    Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);
    double rightDistanceMeters = DuckGearUtil.encoderTicksToMeters(MainRightMotorBack.getSelectedSensorPosition(),
    Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);

    if (Robot.isReal()) {
      odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()), leftDistanceMeters, rightDistanceMeters);
    } else {
      odometry.update(Rotation2d.fromDegrees(-robotDriveSim.getHeading().getDegrees()), leftDistanceMeters, rightDistanceMeters); //since no native support from nav x
    }
    Pose2d previousPose = odometry.getEstimatedPosition();
    field.setRobotPose(previousPose);
  }

  @Override
  public void simulationPeriodic() {
    // For the motor master which is inverted, you'll need to invert it manually (ie
    // with a negative sign) here when fetching any data
    // CTRE doesn't support setInverted() for simulation

    leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    robotDriveSim.setInputs(leftMotorSim.getMotorOutputLeadVoltage(),
        -rightMotorSim.getMotorOutputLeadVoltage());
    // The roboRIO updates at 50hz so you want to match what it actually is in
    // simulation to get accurate simulations
    robotDriveSim.update(0.02);

    // Update sensors
    leftMotorSim
        .setIntegratedSensorRawPosition((int) DuckGearUtil.metersToEncoderTicks(robotDriveSim.getLeftPositionMeters(),
            Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
    leftMotorSim.setIntegratedSensorVelocity(
        (int) DuckGearUtil.metersPerSecondToEncoderTicksPer100ms(robotDriveSim.getLeftVelocityMetersPerSecond(),
        Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
    rightMotorSim
        .setIntegratedSensorRawPosition((int) DuckGearUtil.metersToEncoderTicks(-robotDriveSim.getRightPositionMeters(),
        Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
    rightMotorSim.setIntegratedSensorVelocity(
        (int) DuckGearUtil.metersPerSecondToEncoderTicksPer100ms(-robotDriveSim.getRightVelocityMetersPerSecond(),
        Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(MainLeftMotorBack.getSelectedSensorVelocity(),
            Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters),
        DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(MainRightMotorBack.getSelectedSensorVelocity(),
            Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //flipping voltage for some reason, it works better???
    MainLeftMotorBack.setVoltage(leftVolts);
    MainRightMotorBack.setVoltage(rightVolts);
    robotDrive.feed(); // feed watchdog to prevent error from clogging can bus
  }

  public void resetOdometry(Pose2d pose) {
    gyro.reset();
    resetEncoders();

    odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()),0.,0., pose);
  }
  
  public void resetEncoders() {
    MainLeftMotorBack.setSelectedSensorPosition(0);
    MainRightMotorBack.setSelectedSensorPosition(0);
  }

  public double[] getEncoderPositions() {
    return new double[]{MainLeftMotorBack.getSelectedSensorPosition(0), MainRightMotorBack.getSelectedSensorPosition(0)};
  }
  public double getAvgEncoderPosition() {
        return (MainLeftMotorBack.getSelectedSensorPosition() + MainRightMotorBack.getSelectedSensorPosition())/2.;
  }

}
