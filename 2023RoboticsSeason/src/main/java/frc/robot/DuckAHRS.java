package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;


public class DuckAHRS {
        private AHRS actualGyro;
        double pitchOffset = 0;
        public DuckAHRS() {
                try {
                        /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
                        /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
                        /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
                        actualGyro = new AHRS(SPI.Port.kMXP);
                    } catch (RuntimeException ex ) {
                        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
                    }
        }

        public double getAngle() {
                //yaw (relative to robot)
                if (actualGyro != null) {
                        return (double)(Math.floor(actualGyro.getYaw() * 100.0)/100.0);
                } else {
                        return 0.0;
                }
        }
        public void reset() {
                if (actualGyro != null) {
                        actualGyro.reset();
                }
        }
        public double getPitch() {
                if (actualGyro != null) {
                        return (double)(Math.floor(actualGyro.getPitch() * 100.0)/100.0) - pitchOffset;
                } else {
                        return 0.0;
                }
        }
        public void resetPitch() {
                if (actualGyro != null) {
                        pitchOffset = getPitch();
                }
        }
}
