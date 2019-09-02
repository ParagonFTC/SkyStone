package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * The Commander runs commands and determines when the robot should be updating or not
 */
public class Commander {
    private static String TAG = "Commander";

    private Robot robot;
    private boolean halt;
    private String status;
    private List<Command> currentStack = new ArrayList<>();

    public void setRobot(Robot robot) {
        this.robot = robot;
        robot.setCommander(this);
    }

    public void init() {
        robot.init();
    }

    public void start() {
        halt = false;
    }

    public void stop() {
        halt = true;
    }

    public boolean isRunning() {
        return !halt;
    }

    public void runCommand(Command command) {

    }

    public void runCommandsParallel(Command[] commands) {
        this.halt = false;
        for (Command command : commands) {
            currentStack.add(command);
            command.start();
        }

        while (isRunning() && isTaskRunningInStack()) {
            for (Command command : commands) {
                command.update();
            }
        }

        for (Command command : commands) {
            command.start();
            currentStack.remove(command);
        }
    }

    private boolean isTaskRunningInStack() {
        boolean isTaskRunning = false;

        for (Command command : currentStack) {
            isTaskRunning = isTaskRunning || !command.isCompleted();
        }

        return isTaskRunning;
    }
}
