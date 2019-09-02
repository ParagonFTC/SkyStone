package org.firstinspires.ftc.teamcode;

/**
 * The Command interface represents a certain action performed by the robot
 */
public interface Command {
    /**
     * Called once whenever the command is run, typically used to initialize things
     */
    void start();

    /**
     * Called when the command is run until isCompleted() returns true
     */
    void update();

    /**
     * Called once after the command is run once isCompleted() returns true
     */
    void stop();

    /**
     * Tells the Commander if the command is completed
     *
     * @return if the command is completed
     */
    boolean isCompleted();
}
