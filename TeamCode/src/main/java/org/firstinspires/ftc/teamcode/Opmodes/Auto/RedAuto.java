package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous
public class RedAuto extends SkystoneAutoOpMode {
    public static final double STONE_WIDTH = 49;
    public static double RIGHT_STONE_X = 158;
    public static double CENTER_STONE_X = 98;
    public static double LEFT_STONE_X = 38;

    @Override
    protected SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (Math.abs(RIGHT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.RIGHT;
        else if (Math.abs(CENTER_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.CENTER;
        else if (Math.abs(LEFT_STONE_X - xPos) < STONE_WIDTH/2) return SkystonePosition.LEFT;
        return SkystonePosition.RIGHT;
    }

    @Override
    protected void setup() {
        robot.drive.disengageHooks();
    }

    @Override
    protected void run(SkystonePosition position) {
        switch (position) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case CENTER:
                break;
        }
    }
}
