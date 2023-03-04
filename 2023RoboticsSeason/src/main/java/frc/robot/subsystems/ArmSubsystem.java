package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveArm;

public class ArmSubsystem extends SubsystemBase{
        
        private double m_armKp = 0;
        private double m_armKg = 0;
        //private double motorSpeed = 0;
        public CANSparkMax armMotor = new CANSparkMax(Constants.CAN.Arm.armMotor, MotorType.kBrushless); 
        public RelativeEncoder armEncoder = armMotor.getEncoder(); //in terms of revolutions
        SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 
                Constants.Arm.gearing, SingleJointedArmSim.estimateMOI(Constants.Arm.armLengthMeters, Constants.Arm.armMassKg), 
                Constants.Arm.armLengthMeters, -628, 628, true);

        private final PIDController m_controller = new PIDController(m_armKp, 0, 0); //0.02 works well for kP in sim
        private ArmFeedforward feedforward = new ArmFeedforward(0.0, m_armKg, 0.0); //0.109 works well for kG in sim
        private double m_armSetpointDegrees = 0;

        private final Mechanism2d m_mech2d = new Mechanism2d(2, 2);
        
        private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 1, 1);
        private final MechanismLigament2d m_armTower =
                m_armPivot.append(new MechanismLigament2d("ArmTower", 1, -90));
        private final MechanismLigament2d m_arm =
                m_armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        1,
                        Units.radiansToDegrees(armSim.getAngleRads()),
                        6,
                        new Color8Bit(Color.kYellow)));
        double currentMotorEffort = 0.0;
        public ArmSubsystem(){
                this.setDefaultCommand(new DriveArm(this));
                Preferences.setDouble(Constants.Arm.kArmPositionKey, 0.0);
                SmartDashboard.putData("Arm Sim", m_mech2d);
                m_armTower.setColor(new Color8Bit(Color.kBlue));

                armMotor.clearFaults();
                armMotor.restoreFactoryDefaults();
                armMotor.disableVoltageCompensation();
                armMotor.setInverted(true);
                armMotor.setIdleMode(IdleMode.kBrake);
                armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(Constants.Arm.maxAngle * Constants.Arm.gearing));
                armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(Constants.Arm.minAngle * Constants.Arm.gearing));
                armMotor.burnFlash();

                armEncoder.setPosition(0);

                Preferences.initDouble(Constants.Arm.kArmPositionKey, m_armSetpointDegrees);
                Preferences.initDouble(Constants.Arm.kArmPKey, m_armKp);
                Preferences.initDouble(Constants.Arm.kArmGKey, m_armKg);
        }

        @Override
        public void periodic() {
                m_armSetpointDegrees = MathUtil.clamp(m_armSetpointDegrees, Constants.Arm.minAngle, Constants.Arm.maxAngle);
                double effort = feedforward.calculate(Units.degreesToRadians(m_armSetpointDegrees), 0) + m_controller.calculate(
                        (armEncoder.getPosition() / Constants.Arm.gearing) * 360.0,
                        m_armSetpointDegrees
                );
                //SmartDashboard.putNumber("current target", effort);
                //SmartDashboard.putNumber("encoder position", armEncoder.getPosition());
                effort = MathUtil.clamp(effort, -1, 1);
                currentMotorEffort = effort;
                armMotor.set(effort); //used to be effort
        }
        public void commandAngle(double angle) {
                angle %= 360.;
                m_armSetpointDegrees = angle;
                Preferences.setDouble(Constants.Arm.kArmPositionKey, angle);
        }

        @Override
        public void simulationPeriodic() {
                // In this method, we update our simulation of what our arm is doing
                // First, we set our "inputs" (voltages)
                armSim.setInput(currentMotorEffort * RobotController.getBatteryVoltage());
            
                // Next, we update it. The standard loop time is 20ms.
                armSim.update(0.020);
            
                // Finally, we set our simulated encoder's readings and simulated battery voltage
                armEncoder.setPosition((int) ((armSim.getAngleRads() / 6.28) * Constants.Arm.gearing));
                // SimBattery estimates loaded battery voltages
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
            
                // Update the Mechanism Arm angle based on the simulated arm angle
                m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
        }     

        public void loadPreferences() {
                m_armSetpointDegrees = Preferences.getDouble(Constants.Arm.kArmPositionKey, m_armSetpointDegrees);
                if (m_armKp != Preferences.getDouble(Constants.Arm.kArmPKey, m_armKp)) {
                  m_armKp = Preferences.getDouble(Constants.Arm.kArmPKey, m_armKp);
                  m_controller.setP(m_armKp);
                }
                if(m_armKg != Preferences.getDouble(Constants.Arm.kArmGKey, m_armKg)){
                        m_armKg = Preferences.getDouble(Constants.Arm.kArmGKey, m_armKg);
                        feedforward = new ArmFeedforward(0, m_armKg, 0);
                }
        }

        public double getAngle(){
                return m_armSetpointDegrees;
        }

	public void resetAngle(double angle) {
                Preferences.setDouble(Constants.Arm.kArmPositionKey, angle);
	}
}
