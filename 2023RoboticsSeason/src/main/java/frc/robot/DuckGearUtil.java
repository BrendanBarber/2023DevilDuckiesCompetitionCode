package frc.robot;

public class DuckGearUtil {
  //Gear ratio must be a reduction, CPR means ticks per revolution
  
  public static double encoderTicksToMeters(double ticks, double gearRatio, double CPR, double wheelRadiusMeters) {
    double numAxleRotations = ticks/CPR;
    double numRotationsOnDriverGear = numAxleRotations/gearRatio;
    return numRotationsOnDriverGear*2*Math.PI*wheelRadiusMeters;
  }
  public static double metersToEncoderTicks(double meters, double gearRatio, double CPR, double wheelRadiusMeters) {
   return (meters/(2*Math.PI*wheelRadiusMeters))*gearRatio*CPR;
  }
  public static double metersPerSecondToEncoderTicksPer100ms(double metersPerSecond, double gearRatio, double CPR, double wheelRadiusMeters) {
    metersPerSecond /= 10.0;
    return metersToEncoderTicks(metersPerSecond, gearRatio, CPR, wheelRadiusMeters);
  }
  public static double EncoderTicksPer100msToMetersPerSecond(double ticksPer100ms, double gearRatio, double CPR, double wheelRadiusMeters) {
    return encoderTicksToMeters(ticksPer100ms*10.0, gearRatio, CPR, wheelRadiusMeters);
  }
  public static double RPMToEncoderTicksPer100ms(double rpm, double gearRatio, double CPR) {
    return ((rpm/60.0) * CPR * gearRatio) / 10.0;
  }

public static double EncoderTicksPer100msToRPM(double ticksPer100ms, double gearRatio, double CPR) {
    return (((ticksPer100ms*10.0)/CPR)/gearRatio) * 60.0;
}
}