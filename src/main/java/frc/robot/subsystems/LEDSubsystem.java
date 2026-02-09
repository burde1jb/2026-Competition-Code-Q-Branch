package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    Spark LED;
    // private LaserCan lc;

    public LEDSubsystem() {
        LED = new Spark(0);
        // lc = new LaserCan(0);
    }

    // public boolean lcgetMeasurement() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
    //     if (measurement != null && measurement.distance_mm < 30){
    //         return true;
    //     }
    //     else false;
        
    // }

    public void set(double powerInput) {
        LED.set(powerInput);
    }
}