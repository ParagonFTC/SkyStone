package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Util.AxesSigns;
import org.firstinspires.ftc.teamcode.Util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.Util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.Util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveConstants.kStatic;

@Config
public class RoadRunnerDriveTest extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    private ExpansionHubEx hub1, hub2;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    private ExpansionHubServo leftHook, rightHook;

    public static double leftHookEngagedPosition = 0.4;
    public static double rightHookEngagedPosition = 0.6;
    public static double leftHookDisengagedPosition = 1;
    public static double rightHookDisengagedPosition = 0;

    private ExpansionHubMotor intakeLeft, intakeRight;
    private ExpansionHubServo pusher;

    public static double armPosition = 0;
    public static double disarmPosition = 0.8;

    private ExpansionHubMotor lift;
    public static final double LIFT_TICKS_PER_REV = 537.6;

    public RoadRunnerDriveTest(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub1.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftBack");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightBack");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: tune velocity PID Coefficients

        leftHook = hardwareMap.get(ExpansionHubServo.class, "leftHook");
        rightHook = hardwareMap.get(ExpansionHubServo.class, "rightHook");

        intakeLeft = hardwareMap.get(ExpansionHubMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(ExpansionHubMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher = hardwareMap.get(ExpansionHubServo.class, "pusher");

        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);
                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        RevBulkData bulkData1 = hub1.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        if (bulkData1 == null || bulkData2 == null) {
            return  Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftFront)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftRear)));
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorVelocity(rightRear)));
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorVelocity(rightFront)));
        return wheelPositions;
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData1 = hub1.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        if (bulkData1 == null || bulkData2 == null) {
            return  Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftFront)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftRear)));
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorCurrentPosition(rightRear)));
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorCurrentPosition(rightFront)));
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    protected double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void engageHooks() {
        leftHook.setPosition(leftHookEngagedPosition);
        rightHook.setPosition(rightHookEngagedPosition);
    }

    public void disengageHooks() {
        leftHook.setPosition(leftHookDisengagedPosition);
        rightHook.setPosition(rightHookDisengagedPosition);
    }

    public void armPusher() {
        pusher.setPosition(armPosition);
    }

    public void disarmPusher() {
        pusher.setPosition(disarmPosition);
    }

    public void setPusherPosition(double position) {
        pusher.setPosition(position);
    }

    public void setIntakePower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void raiseLift() {
        lift.setTargetPosition(1010);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        while (!Thread.currentThread().isInterrupted() && lift.isBusy()) {
            update();
        }
    }

    public void lowerLift() {
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        /*
        while (!Thread.currentThread().isInterrupted() && lift.isBusy()) {
            update();
        }
        */
    }
}
