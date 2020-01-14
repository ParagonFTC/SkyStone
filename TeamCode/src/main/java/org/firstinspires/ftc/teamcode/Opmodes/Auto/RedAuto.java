package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class RedAuto extends SkystoneAutoOpMode {
    @Override
    protected SkystonePosition getSkystonePosition() {
        double xPos = detector.getScreenPosition().x;
        if (xPos < 107) return SkystonePosition.LEFT;
        if (107 < xPos && xPos < 213) return SkystonePosition.CENTER;
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
