package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class Outtake implements Subsystem {
    ExpansionHubMotor lift;
    private double liftPower;

    ExpansionHubServo grabber;
    public static double grabberArmPosition = 0.8;
    public static double grabberDisarmPosition = 0.3;

    ExpansionHubServo wrist;
    public static double wristArmPosition = 0;
    public static double wristDisarmPosition = 1;

    CRServo thing;
    private double thingPower;

    public Outtake(HardwareMap hardwareMap) {
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        grabber = hardwareMap.get(ExpansionHubServo.class, "grabber");
        wrist = hardwareMap.get(ExpansionHubServo.class, "wrist");
        thing = hardwareMap.get(CRServo.class, "thing");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLiftPower(double power) {
        //if (power < 0) power /= 2;
        if (power == 0) {
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            power = 0.5;
        } else {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        liftPower = power;
    }

    public void armGrabber() {
        grabber.setPosition(grabberArmPosition);
    }

    public void disarmGrabber() {
        grabber.setPosition(grabberDisarmPosition);
    }

    public void armWrist() {
        wrist.setPosition(wristArmPosition);
    }

    public void disarmWrist() {
        wrist.setPosition(wristDisarmPosition);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public void setThingPower(double power) {
        thingPower = power;
    }
    @Override
    public void update() {
        lift.setPower(liftPower);

        thing.setPower(thingPower);
    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
