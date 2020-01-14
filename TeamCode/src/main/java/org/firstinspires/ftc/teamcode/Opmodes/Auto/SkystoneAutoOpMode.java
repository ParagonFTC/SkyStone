package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Vision.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class SkystoneAutoOpMode extends LinearOpMode {
    protected Robot robot;

    protected CustomSkystoneDetector detector;
    private OpenCvCamera camera;

    public enum SkystonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private SkystonePosition position;

    protected Telemetry telemetry;

    protected abstract SkystonePosition getSkystonePosition();
    protected abstract void setup();
    protected abstract void run(SkystonePosition position);
    @Override
    public final void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this);
        robot.start();
        detector = new CustomSkystoneDetector();
        detector.useDefaults();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        setup();

        while (!isStarted()) {
            position = getSkystonePosition();
            telemetry.addData("Skystone Position", position);
        }

        AutoTransitioner.transitionOnStop(this, "Teleop");

        waitForStart();

        if (isStopRequested()) return;

        run(position);
    }
}
