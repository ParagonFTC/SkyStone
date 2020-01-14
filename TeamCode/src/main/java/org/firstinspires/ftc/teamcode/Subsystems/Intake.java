package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class Intake implements Subsystem {
    ExpansionHubMotor intakeLeft, intakeRight;
    public static double defaultSpeed = 1.0;
    private double speed;
    private boolean active;

    public static double armPosition = 0.4;
    public static double disarmPosition = 0.8;
    private double pusherPosition;

    public Intake(HardwareMap hardwareMap) {
        intakeLeft = hardwareMap.get(ExpansionHubMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(ExpansionHubMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setSpeed(double power) {
        this.speed = power;
        active = speed != 0.0;
    }

    public double getSpeed() {
        return speed;
    }

    public void toggleIntake() {
        if (!active) {
            setSpeed(defaultSpeed);
        } else {
            setSpeed(0);
        }
    }

    @Override
    public void update() {
        intakeLeft.setPower(speed);
        intakeRight.setPower(speed);
    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
