package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DuckAHRS;
import frc.robot.DuckGearUtil;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;

public class DrivetrainSubsystem extends SubsystemBase {
  public final DifferentialDrive robotDrive;
  private final DifferentialDrivetrainSim robotDriveSim;
  private RobotContainer m_robotContainer;

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
  private final PhotonCamera camera;
  public AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimator;
  public boolean isCurrentLimited = false;
  private double gyroPitchkP = 0.037;
  private double forwardMovementkP = 0.00005;
  private double globalRotationkP = 0.0005;

  //proportional controllers only gang uwu
  public PIDController pidGyroPitch = new PIDController(gyroPitchkP, 0.0, 0.0);
  public PIDController pidMovement = new PIDController(forwardMovementkP, 0.0, 0.0);
  public PIDController pidGlobalRotation = new PIDController(globalRotationkP, 0.0, 0.0);

  public DrivetrainSubsystem(RobotContainer robotContainer) {
    //constructor gets ran at robotInit()
    this.setDefaultCommand(new DriveCommand(this));
    this.m_robotContainer = robotContainer;
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
        Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-chargedup.json"
      );
    } catch(IOException e) {
      System.out.println("couldnt load field image :(");
    }
    camera = new PhotonCamera(Constants.CameraCharacteristics.photonVisionName);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.CameraCharacteristics.robotToCamMeters);

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

    MainLeftMotorBack.config_kP(0, 0.2);
    MainRightMotorBack.config_kP(0, 0.2);

    // Slave the front motors to their respective back motors
    MainLeftMotorFront.follow(MainLeftMotorBack);
    MainRightMotorFront.follow(MainRightMotorBack);
    leftTopMotor.follow(MainLeftMotorBack);
    rightTopMotor.follow(MainRightMotorBack);

    // Disable voltage compensation, it's bad to be compensating voltage for a
    // system which draws loads of amps

    MainLeftMotorBack.configOpenloopRamp(0.25);
    MainRightMotorBack.configOpenloopRamp(0.25);
    MainLeftMotorFront.configOpenloopRamp(0.25);
    MainRightMotorFront.configOpenloopRamp(0.25);
    leftTopMotor.configOpenloopRamp(0.25);
    rightTopMotor.configOpenloopRamp(0.25);
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
    Preferences.setDouble(Constants.DrivetrainCharacteristics.gyroPitchPGainKey, gyroPitchkP);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.movementPGainKey, forwardMovementkP);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.globalRotationPGainKey, globalRotationkP);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.speedScaleKey, Constants.DrivetrainCharacteristics.speedScale);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.turnSpeedKey, Constants.DrivetrainCharacteristics.turnSpeedScale);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.maxAutoVelocityMetersKey, Constants.DrivetrainCharacteristics.maxAutoVelocityMeters);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.maxAutoAccelerationMetersKey, Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.kPKey, Constants.DrivetrainCharacteristics.kP);
    Preferences.setDouble(Constants.DrivetrainCharacteristics.ksAngularKey, Constants.DrivetrainCharacteristics.kSAngular);
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
    if (previousPose != null) {
        photonPoseEstimator.setReferencePose(previousPose);
    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
    if (result.isPresent()) {
        Pose2d estimatedPoseMeters = result.get().estimatedPose.toPose2d();
        SmartDashboard.putString("estimated pose meters", estimatedPoseMeters.toString());
        odometry.addVisionMeasurement(estimatedPoseMeters, currentTime);
    }
    }
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
  public void loadPreferences() {
        if (gyroPitchkP != Preferences.getDouble(Constants.DrivetrainCharacteristics.gyroPitchPGainKey, gyroPitchkP)) {
                gyroPitchkP = Preferences.getDouble(Constants.DrivetrainCharacteristics.gyroPitchPGainKey, gyroPitchkP);
                pidGyroPitch.setP(gyroPitchkP);
        }
        if (forwardMovementkP != Preferences.getDouble(Constants.DrivetrainCharacteristics.movementPGainKey, forwardMovementkP)) {
                forwardMovementkP = Preferences.getDouble(Constants.DrivetrainCharacteristics.movementPGainKey, forwardMovementkP);
                pidMovement.setP(forwardMovementkP);
        }
        if (globalRotationkP != Preferences.getDouble(Constants.DrivetrainCharacteristics.globalRotationPGainKey, globalRotationkP)) {
          globalRotationkP = Preferences.getDouble(Constants.DrivetrainCharacteristics.globalRotationPGainKey, forwardMovementkP);
          pidGlobalRotation.setP(globalRotationkP);
        }
        if(Constants.DrivetrainCharacteristics.kP != Preferences.getDouble(Constants.DrivetrainCharacteristics.kPKey, Constants.DrivetrainCharacteristics.kP)){
          Constants.DrivetrainCharacteristics.kP = Preferences.getDouble(Constants.DrivetrainCharacteristics.kPKey, Constants.DrivetrainCharacteristics.kP);      
          Autos.pushAutosToDashboard(m_robotContainer.autonomousMode, m_robotContainer.getDrivetrainSubsystem(), m_robotContainer.getClawSubsystem());
        }
      
        Constants.DrivetrainCharacteristics.kSAngular = Preferences.getDouble(Constants.DrivetrainCharacteristics.ksAngularKey, Constants.DrivetrainCharacteristics.kSAngular);
        Constants.DrivetrainCharacteristics.speedScale = Preferences.getDouble(Constants.DrivetrainCharacteristics.speedScaleKey, Constants.DrivetrainCharacteristics.speedScale);
        Constants.DrivetrainCharacteristics.turnSpeedScale = Preferences.getDouble(Constants.DrivetrainCharacteristics.turnSpeedKey, Constants.DrivetrainCharacteristics.turnSpeedScale);
        Constants.DrivetrainCharacteristics.maxAutoVelocityMeters = Preferences.getDouble(Constants.DrivetrainCharacteristics.maxAutoVelocityMetersKey, Constants.DrivetrainCharacteristics.maxAutoVelocityMeters);
        Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters = Preferences.getDouble(Constants.DrivetrainCharacteristics.maxAutoAccelerationMetersKey, Constants.DrivetrainCharacteristics.maxAutoAccelerationMeters);
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

  public PhotonCamera getCamera(){
        return camera;
  }

  public void setCurrentLimit(boolean isCurrentLimited) {
    int currentLimitAmp = 40;
    int thresholdAmp = 2;
    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(isCurrentLimited, currentLimitAmp, currentLimitAmp+thresholdAmp, 0.5);
    
    MainLeftMotorBack.configSupplyCurrentLimit(supplyLimit, 100);
    MainRightMotorBack.configSupplyCurrentLimit(supplyLimit, 100);
  }
}
