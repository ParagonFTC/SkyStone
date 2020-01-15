package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
public class Outtake2 implements Subsystem {
    ExpansionHubMotor lift;
    private int liftPosition;
    public static final double LIFT_ITERATION = 4.0;
    public static final double PULLEY_RADIUS = 1.259843/2;
    public static final double TICKS_PER_REV = 537.6;

    ExpansionHubServo wristLeft;
    ExpansionHubServo wristRight;
    public static double wristDeployPosition = 0.9;
    public static double wristGrabPosition = 0.01;
    public static double wristIdlePosition = 0.15;
    public static double wristLiftPosition = 0.3;
    public static double wristDelay = 0.5;

    ExpansionHubServo grabber;
    public static double grabberArmPosition = 1;
    public static double grabberDisarmPosition = 0.7;

    public enum Mode {
        OPEN_LOOP,
        RUN_TO_POSITION
    }

    public enum WristPosition {
        GRAB,
        LIFT,
        DEPLOY,
        IDLE
    }

    private Mode mode;
    private WristPosition wristPosition;
    private WristPosition lastPosition;
    private boolean liftRaised;

    NanoClock clock;
    double startTimestamp;

    public Outtake2(HardwareMap hardwareMap) {
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wristLeft = hardwareMap.get(ExpansionHubServo.class, "wristLeft");
        wristRight = hardwareMap.get(ExpansionHubServo.class, "wristRight");
        wristLeft.setPwmRange(new PwmControl.PwmRange(500,2200));
        wristRight.setPwmRange(new PwmControl.PwmRange(500,2200));

        grabber = hardwareMap.get(ExpansionHubServo.class, "grabber");

        mode = Mode.OPEN_LOOP;
        wristPosition = WristPosition.IDLE;
        liftRaised = false;

        clock = NanoClock.system();
        liftPosition = 0;
    }

    public void setLiftPower(double power) {
        if (mode == Mode.OPEN_LOOP) {
            lift.setPower(power);
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setWristPosition(double position) {
        wristLeft.setPosition(position);
        wristRight.setPosition(1-position);
    }

    public WristPosition getWristPosition() {
        return wristPosition;
    }

    public void armGrabber() {
        grabber.setPosition(grabberArmPosition);
    }

    public void disarmGrabber() {
        grabber.setPosition(grabberDisarmPosition);
    }

    public void raiseLift() {
        mode = Mode.RUN_TO_POSITION;
        if (liftPosition != 7) lift.setTargetPosition(-(liftPosition - 1) * encoderInchesToTicks(LIFT_ITERATION));
        if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        liftRaised = true;
    }

    public void lowerLift() {
        mode = Mode.RUN_TO_POSITION;
        lift.setTargetPosition(0);
        if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION)lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRaised = false;
        lift.setPower(1);
    }

    public void liftPositionUp() {
        if (liftPosition != 7) liftPosition ++;
        raiseLift();
    }

    public void liftPositionDown() {
        if (liftPosition != 1) liftPosition --;
        raiseLift();
    }

    public void cycleWrist() {
        switch (wristPosition) {
            case IDLE:
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.GRAB;
                break;
            case GRAB:
                liftPositionUp();
                wristPosition = WristPosition.LIFT;
                break;
            case LIFT:
                wristPosition = WristPosition.DEPLOY;
                break;
            case DEPLOY:
                startTimestamp = clock.seconds();
                wristPosition = WristPosition.IDLE;
                break;
        }
    }

    public double getLiftPosition() {
        return liftPosition * 4;
    }

    public static double encoderTicksToInches (double ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int encoderInchesToTicks (double inches) {
        return (int) (inches * TICKS_PER_REV / PULLEY_RADIUS / 2 / Math.PI);
    }

    @Override
    public void update() {
        switch (wristPosition) {
            case IDLE:
                disarmGrabber();
                setWristPosition(wristIdlePosition);
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    lowerLift();
                }
                break;
            case GRAB:
                setWristPosition(wristGrabPosition);
                if (clock.seconds() > (startTimestamp + wristDelay)) {
                    armGrabber();
                }
                break;
            case LIFT:
                setWristPosition(wristLiftPosition);
                raiseLift();
                break;
            case DEPLOY:
                if (liftPosition == 7) setWristPosition(0.7);
                else setWristPosition(wristDeployPosition);
                break;
        }
        if (!lift.isBusy()) mode = Mode.OPEN_LOOP;
        lastPosition = wristPosition;
    }

    @Override
    public boolean isBusy() {
        return mode == Mode.OPEN_LOOP;
    }
}
