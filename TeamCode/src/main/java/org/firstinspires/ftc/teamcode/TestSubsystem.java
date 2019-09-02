package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestSubsystem implements Subsystem {
    private DcMotor motor;
    private HardwareMap map;
    private double motorPower;

    public TestSubsystem(HardwareMap map) {
        this.map = map;
    }
    @Override
    public void init() {
        motor = map.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorPower(double power) {
        motorPower = power;
    }

    @Override
    public void update() {
        motor.setPower(motorPower);
    }
}
