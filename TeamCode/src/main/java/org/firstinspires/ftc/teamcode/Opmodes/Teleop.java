package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;
/*
    DcMotor lift;
    CRServo thing;
    Servo wrist;
    Servo grabber; */
    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
        /*
        lift = hardwareMap.dcMotor.get("lift");
        thing = hardwareMap.crservo.get("thing");
        wrist = hardwareMap.servo.get("wrist");
        grabber = hardwareMap.servo.get("grabber"); */
        telemetry.addLine("God is dead and we have killed him");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.drive.setDrivePower(new Pose2d(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x));
        robot.intake.setSpeed(gamepad1.left_trigger);
        if (gamepad1.left_bumper) {
            robot.intake.setSpeed(-1);
        }
        /*
        lift.setPower(gamepad1.right_trigger);
        if (gamepad1.right_bumper) lift.setPower(-1);
        else lift.setPower(-0.1); */
        telemetry.addData("intake power", robot.intake.getSpeed());
        if (gamepad1.a) robot.drive.engageHooks();
        if (gamepad1.b) robot.drive.disengageHooks();
        if (gamepad1.x) robot.intake.armPusher();
        if (gamepad1.y) robot.intake.disarmPusher();
    }
}
