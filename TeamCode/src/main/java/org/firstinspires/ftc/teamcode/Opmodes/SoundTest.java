package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sound Test", group = "test")
public class SoundTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int soundID = hardwareMap.appContext.getResources().getIdentifier("toad_here_we_go","raw",hardwareMap.appContext.getPackageName());
        waitForStart();
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        while (!isStopRequested()) {
            idle();
        }
    }
}
