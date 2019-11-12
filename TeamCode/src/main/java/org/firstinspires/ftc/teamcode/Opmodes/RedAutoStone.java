package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;

@Autonomous
public class RedAutoStone extends LinearOpMode {

    @Override
    public void runOpMode() {
        RoadRunnerDriveTest drive = new  RoadRunnerDriveTest(hardwareMap);

        drive.disengageHooks();
        drive.setPusherPosition(0.5);

        telemetry.addLine("initialization complete");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-24, -30, Math.PI/2))
                        .build()
        );

        drive.disarmPusher();
        drive.setIntakePower(1);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-42,-70,Math.PI/4))
                        .build()
        );
        sleep(2000);

        drive.setIntakePower(0);
        drive.setPusherPosition(0.5);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-25,-30,Math.PI/2))
                        .splineTo(new Pose2d(-25,-20,Math.PI/2))
                        .splineTo(new Pose2d(-31,32,Math.PI))
                        .splineTo(new Pose2d(-44,32,Math.PI))
                        .build()
        );

        drive.engageHooks();
        sleep(1000);

        drive.raiseLift();
        drive.setIntakePower(1);
        drive.armPusher();
        sleep(1000);
        drive.disarmPusher();
        sleep(500);
        drive.armPusher();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(12,0,Math.PI/3))
                        .build()
        );

        drive.disengageHooks();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );

        drive.disarmPusher();
        drive.lowerLift();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-6,-30,Math.PI/2))
                        .build()
        );
    }
}
