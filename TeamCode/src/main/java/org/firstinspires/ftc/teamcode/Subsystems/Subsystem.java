package org.firstinspires.ftc.teamcode.Subsystems;



public interface Subsystem {

    /**
     * Ocean man take me by the hand take me to the land
     * updates the hardware in the subsystem
     */
    void update();

    /**
     * Ocean man the voyage to the corner of the globe is a real trip
     * @return whether the subsystem is busy or not (duh)
     */
    boolean isBusy();
}
