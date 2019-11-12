package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;
import org.firstinspires.ftc.teamcode.Util.StickyGamepad;

@Autonomous
public class AutoPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerDriveTest drive = new RoadRunnerDriveTest(hardwareMap);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        int delay = 0;

        drive.disengageHooks();
        drive.setPusherPosition(0.5);

        telemetry.addLine("initialization complete");
        telemetry.update();
        while (!isStarted()) {
            if (stickyGamepad1.dpad_up) delay ++;
            if (stickyGamepad1.dpad_down && delay != 0) delay --;
            telemetry.addData("delay", delay);
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        sleep(delay * 1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(36)
                        .build()
        );
    }
}
