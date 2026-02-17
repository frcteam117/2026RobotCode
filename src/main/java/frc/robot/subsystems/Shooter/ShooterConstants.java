package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    // add like max rpm and constants for auto shooting/turning/hood
    public static final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();
    // position from hub (meters) : angle of hood/speed of motor
    
    public ShooterConstants() { // last years field was wxh 17x7, check current field dimensions!!!
        lerpTable.put(0.0, 0.3);
        lerpTable.put(2.0, 0.35);
        lerpTable.put(5.0, 0.5);
        lerpTable.put(10.0, 0.6);
        lerpTable.put(20.0, 0.7);
    }
    // ...
    //double result = table.get(1.5); // Returns 20.0
}
