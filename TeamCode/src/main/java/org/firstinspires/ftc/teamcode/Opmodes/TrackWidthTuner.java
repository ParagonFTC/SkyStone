package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Subsystems.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerDriveTest;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@Autonomous
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RoadRunnerDriveTest drive = new RoadRunnerDriveTest(hardwareMap);

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        MovingStatistics trackwidthStatistics = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i ++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turn(Math.toRadians(ANGLE));

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackwidth = DriveConstants.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackwidthStatistics.add(trackwidth);

            sleep(DELAY);
        }

        telemetry.log().clear();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackwidthStatistics.getMean(),
                trackwidthStatistics.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
