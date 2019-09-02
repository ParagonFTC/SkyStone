package org.firstinspires.ftc.teamcode;

/**
 * The Subsystem interface represents a physical component of a robot. These
 * can be a drivetrain, a claw, etc.
 */

public interface Subsystem {
    /**
     * Initializes the hardware of the subsystem, is called by the Robot
     */
    void init();

    /**
     * This method is called by the Robot while the Commander is active
     */
    void update();
}
