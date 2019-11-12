package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;

@Autonomous
public class RedAutoFoundation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerDriveTest drive = new RoadRunnerDriveTest(hardwareMap);

        drive.disengageHooks();
        drive.setPusherPosition(0.5);

        telemetry.addLine("initialization complete");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        sleep(15000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(48,-18,0))
                        .build()
        );

        drive.engageHooks();
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,0,-Math.PI/2))
                        .build()
        );

        drive.disengageHooks();
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(12)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,30, -Math.PI/2))
                        .build()
        );
    }
}
