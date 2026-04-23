// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShootingTables {

    public static InterpolatingDoubleTreeMap FlywheelMap = new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap ExitVelocityMap = new InterpolatingDoubleTreeMap();

    static{
        /*Close Range shot */
        FlywheelMap.put(2.5, 2400.0);
        ExitVelocityMap.put(2.5, 8.0);

        /*Mid Range shot */
        FlywheelMap.put(3.00, 2520.0);
        ExitVelocityMap.put(3.00, 33.0);

        /*Mid Range shot */
        FlywheelMap.put(4.00, 2820.0);
        ExitVelocityMap.put(4.00, 16.0);

        /*Long Range shot */
        FlywheelMap.put(5.53, 3200.0);
        ExitVelocityMap.put(5.53, 26.0);
    }
}
