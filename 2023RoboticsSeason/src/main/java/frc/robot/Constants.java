// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        // All constants are in SI base units

        public static class OperatorConstants {
                public static final int port = 1;
                public static final int movementAxis = 1;
                public static final double movementDeadband = 0.5;
        }

        public static class DriverConstants {
                public static final int port = 0;
                public static final double deadbandLeftJoystick = 0.05;
                public static final double deadbandRightJoystick = deadbandLeftJoystick;
                public static final int ForwardDriveAxis = 1;//3;
                public static final int TurningDriveAxis = 2;
        }

        public static class CAN {
                public static class Drivetrain {
                        // configured for heimerdinger
                        public static final int BL = 5;
                        public static final int BR = 4;
                        public static final int FL = 3;
                        public static final int FR = 7;
                        public static final int TL = 2;
                        public static final int TR = 6;
                }

                public static class Arm {
                        public static final int armMotor = 8;
                }
                public static class Claw {
                        public static final int motor = 9;
                }
        }

        public static class Arm {
                public static final double gearing = 3.67*3.67*3.67;
                public static final double armMassKg = 10.;
                public static final double armLengthMeters = 1.;
                public static final String kArmPositionKey = "arm position";
                public static final String kArmPKey = "arm kp";
                public static final String kArmGKey = "arm kg";
                public static final double minAngle = -90.0;
                public static final double maxAngle = 90.0;
        }

        public static class DrivetrainCharacteristics {
                public static final double trackWidthMeters = Units.inchesToMeters(18);
                public static final double gearing = 6.4;
                public static final double wheelRadiusMeters = Units.inchesToMeters(2);
                public static final double rampPGain = 0.1;
                public static final String gyroPitchPGainKey = "gyro pitch kP";
                public static final String movementPGainKey = "movement kP";
                public static final String globalRotationPGainKey = "global rotation kP";
                public static final String ksAngularKey = "rotation voltage min";
                public static double kSAngular = -0.1;
                public static double kS = 0.11429;
                public static double kV = 2.2146;
                public static double kA = 0.18994;
                public static String kPKey = "drive kP";
                public static double kP = 0.37182;//2.9076;
                public static String maxAutoVelocityMetersKey = "max auto velocity (meters)";
                public static double maxAutoVelocityMeters = 1;
                public static String maxAutoAccelerationMetersKey = "max auto acceleration (meters)";
                public static double maxAutoAccelerationMeters = 0.75;
                public static double kD = 0.0;
                public static String speedScaleKey = "drivetrain speed";
                public static double speedScale = 1.0;
                public static String turnSpeedKey = "drivetrain turn speed";
                public static double turnSpeedScale = 0.8;
                public static double deadband = 0.1;
        }

        public static class CameraCharacteristics {
                public static final String photonVisionName = "camera";
                public static final Transform3d robotToCamMeters = new Transform3d(
                                new Translation3d(-Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(15.5)),
                                new Rotation3d(0, Units.degreesToRadians(12), 0));
        }

        public static class AutoTrajectoryFileNames {
                public static final String HIGH_TAXI = "HighTaxiPath";
                public static final String MID_TAXI = "MidTaxiPath";
                public static final String LOW_TAXI = "LowTaxiPath";
                public static final String MID_BALANCE = "MidBalance";
                public static final String CONE_TAXI = "ConeTaxi";
                public static final String DOCK = "DockPath";
        }

        public static class ClawCharacteristics {
                public static final String clawSpeedKey = "claw speed";
                public static final String clawTimeKey = "claw time";
                public static final String clawTimeoutTimeKey = "claw timeout time";
                public static int forwardChannel = 0;
                public static int reverseChannel = 1;
        }

        public static HashMap<String, Command> eventMap = new HashMap<>();
}
