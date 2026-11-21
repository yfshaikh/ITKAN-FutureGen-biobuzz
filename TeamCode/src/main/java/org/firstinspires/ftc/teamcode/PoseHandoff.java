package org.firstinspires.ftc.teamcode;

/*
 * Passes the robot's heading from auton to teleop. Static fields live as long as the Robot
 * Controller app, so a teleop started right after auton can read what auton wrote.
 * NaN = no auton since the app started (or teleop already used it), so teleop re-zeroes instead.
 */
public class PoseHandoff {
    public static double headingRad = Double.NaN;   // Pedro field heading at the end of auton
}
