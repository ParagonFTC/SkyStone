package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;

@TeleOp
public class Teleop2 extends OpMode {
    RoadRunnerDriveTest drive;
    @Override
    public void init() {
        drive = new RoadRunnerDriveTest(hardwareMap);
    }

    @Override
    public void loop() {
        drive.setDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        drive.setIntakePower(-gamepad1.left_trigger);
        if (gamepad1.left_bumper) {
            drive.setIntakePower(1);
        }
    }
}
