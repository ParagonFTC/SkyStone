package org.firstinspires.ftc.teamcode;

public class TestCommand implements Command {
    private TestSubsystem testSubsystem;
    private double power;
    private double duration;
    private double startTimestamp;

    public TestCommand (TestSubsystem testSubsystem, double power, double duration) {
        this.testSubsystem = testSubsystem;
        this.power = power;
        this.duration = duration;
    }

    @Override
    public void start() {
        startTimestamp = System.nanoTime() * Math.pow(10,9);
        testSubsystem.setMotorPower(power);
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        testSubsystem.setMotorPower(0);
    }

    @Override
    public boolean isCompleted() {
        double currentTime = System.nanoTime() * Math.pow(10,9);
        return currentTime - startTimestamp > duration;
    }
}
