package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake2;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp
public class Teleop extends OpMode {
    Robot robot;

    StickyGamepad stickyGamepad1, stickyGamepad2;

    double t = -1;

    ExpansionHubMotor park;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        robot = new Robot(this);
        robot.start();

        telemetry.addLine("God is dead and we have killed him");
        telemetry.update();
        int soundID = hardwareMap.appContext.getResources().getIdentifier("toad_here_we_go","raw",hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

        park = hardwareMap.get(ExpansionHubMotor.class, "park");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        if (stickyGamepad1.right_bumper) t = -t;
        if (stickyGamepad1.right_stick_button) {
            if (Math.abs(t) == 1) t *= 0.3;
            else  t *= 10.0/3;
        }
        if (gamepad1.right_stick_x != 0) robot.drive.setDrivePower(new Pose2d(0,0, Math.abs(t) * gamepad1.right_stick_x));
        else robot.drive.setDrivePower(new Pose2d(t*gamepad1.left_stick_y,-t*gamepad1.left_stick_x,0));
        robot.intake.setSpeed(gamepad1.left_trigger);
        if (gamepad1.left_bumper) {
            robot.intake.setSpeed(-1);
        }

        telemetry.addData("intake power", robot.intake.getSpeed());
        if (gamepad1.a) robot.drive.engageHooks();
        if (gamepad1.b) robot.drive.disengageHooks();

        robot.outtake.setLiftPower(gamepad2.left_stick_y);
        if (stickyGamepad2.dpad_up) robot.outtake.liftPositionUp();
        else if (stickyGamepad2.dpad_down) robot.outtake.liftPositionDown();

        if (gamepad2.a) robot.outtake.armGrabber();
        if (gamepad2.b) robot.outtake.disarmGrabber();
        if (gamepad2.right_bumper) {
            if (gamepad2.y) {
                robot.outtake.setWristPosition(Outtake2.wristGrabPosition);
            } else if (gamepad2.x) {
                robot.outtake.setWristPosition(Outtake2.wristDeployPosition);
            } else {
                robot.outtake.setWristPosition(gamepad2.right_trigger);
            }
        } else if (stickyGamepad2.x) robot.outtake.cycleWrist();
        telemetry.addData("switch mode engaged", t);
        telemetry.addData("wrist position", robot.outtake.getWristPosition());
        park.setPower(gamepad2.left_trigger);
        telemetry.addData("lift position", robot.outtake.getLiftPosition());
        telemetry.addData("dpad up", gamepad2.dpad_up);
        telemetry.addData("dpad down", gamepad2.dpad_down);
        telemetry.addData("lift mode", robot.outtake.getMode());
        if (gamepad2.left_bumper) park.setPower(-1);
    }
}
