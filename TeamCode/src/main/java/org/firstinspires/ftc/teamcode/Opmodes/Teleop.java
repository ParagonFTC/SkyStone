package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;
/*
    DcMotor lift;
    CRServo thing;
    Servo wrist;
    Servo grabber; */

    ExpansionHubServo markerDeploy;
    StickyGamepad stickyGamepad1;

    public static double hold = 0.3;
    public static double release = 1.0;

    double t = -1;
    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        robot = new Robot(this);
        robot.start();
        /*
        lift = hardwareMap.dcMotor.get("lift");
        thing = hardwareMap.crservo.get("thing");
        wrist = hardwareMap.servo.get("wrist");
        grabber = hardwareMap.servo.get("grabber"); */
        markerDeploy = hardwareMap.get(ExpansionHubServo.class, "hold");
        telemetry.addLine("God is dead and we have killed him");
        telemetry.update();
        int soundID = hardwareMap.appContext.getResources().getIdentifier("toad_here_we_go","raw",hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        markerDeploy.setPosition(hold);
        robot.outtake.disarmWrist();
        robot.outtake.disarmGrabber();
        robot.intake.disarmPusher();
    }

    @Override
    public void start() {
        robot.intake.disarmPusher();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        if (stickyGamepad1.right_bumper) t = -t;
        if (stickyGamepad1.right_stick_button) {
            if (Math.abs(t) == 1) t *= 0.3;
            else  t *= 10.0/3;
        }
        if (gamepad1.right_stick_x != 0) robot.drive.setDrivePower(new Pose2d(0,0, Math.abs(t) * gamepad1.right_stick_x));
        else robot.drive.setDrivePower(new Pose2d(t*gamepad1.left_stick_y,t*gamepad1.left_stick_x,0));
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

        robot.outtake.setLiftPower(gamepad2.left_stick_y);
        robot.outtake.setThingPower(gamepad2.right_stick_y);

        if (gamepad2.a) robot.outtake.armGrabber();
        if (gamepad2.b) robot.outtake.disarmGrabber();
        if (gamepad2.x) robot.outtake.armWrist();
        if (gamepad2.y) robot.outtake.disarmWrist();
        if (gamepad1.dpad_down) markerDeploy.setPosition(hold);
        if (gamepad1.dpad_up) markerDeploy.setPosition(release);
        if (gamepad2.right_bumper) robot.outtake.setWristPosition(0.5);
        if (gamepad1.right_trigger > 0.5) robot.intake.setPusherPosition(0);
        telemetry.addData("switch mode engaged", t);
    }
}
