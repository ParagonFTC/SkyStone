package org.firstinspires.ftc.teamcode;

import java.util.HashSet;
import java.util.Set;

/**
 * The robot holds all of the Subsystems and updates them on their own thread
 */
public abstract class Robot {
    private Set<Subsystem> subsystems = new HashSet<>();
    private Commander commander;

    private Runnable hardwareUpdater = new Runnable() {
        @Override
        public void run() {
            while (commander.isRunning()) {
                for (Subsystem subsystem : subsystems) {
                    subsystem.update();
                }
            }
        }
    };

    public void addSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    void setCommander(Commander commander) {
        this.commander = commander;
    }

    public void init() {
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }

        new Thread(hardwareUpdater).start();
    }
}
