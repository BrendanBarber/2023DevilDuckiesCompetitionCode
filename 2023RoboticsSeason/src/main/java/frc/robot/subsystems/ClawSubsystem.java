package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
        public TalonSRX motor = new TalonSRX(Constants.CAN.Claw.motor);
        private double clawSpeed = 0.5;
        private double clawTime = 0.5; //for increment based movement
        public boolean isClosing = false;
        private boolean disabled = false;
        public DigitalInput sonarSensor = new DigitalInput(4);
        public ClawSubsystem() {
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Brake);
                Preferences.initDouble(Constants.ClawCharacteristics.clawSpeedKey, clawSpeed);
                Preferences.initDouble(Constants.ClawCharacteristics.clawTimeKey, clawTime);
        }

        public void periodic(){
                //SmartDashboard.putBoolean("sonar sensor state", sonarSensor.get());
                //SmartDashboard.putNumber("claw time", clawTime);
        }

        public void loadPreferences() {
                clawSpeed = Preferences.getDouble(Constants.ClawCharacteristics.clawSpeedKey, clawSpeed);
                clawTime = Preferences.getDouble(Constants.ClawCharacteristics.clawTimeKey, clawTime);
        }
        public double getClawSpeed() {
                return clawSpeed;
        }
        public double getClawTime(){
                return clawTime;
        }
        public boolean getDisabledState() {
                return disabled;
        }
}
