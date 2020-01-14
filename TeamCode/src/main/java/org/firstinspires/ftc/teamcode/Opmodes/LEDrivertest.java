package org.firstinspires.ftc.teamcode.Opmodes;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.LEDRiver;

@TeleOp
public class LEDrivertest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule hub = hardwareMap.get(LynxModule.class,"Expansion Hub 1");
        try {
            new LynxI2cConfigureChannelCommand(hub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "ledriver");

        waitForStart();

        ledRiver.setMode(LEDRiver.Mode.PATTERN)
                .setColor(0,Color.YELLOW)
                .setColor(1,Color.WHITE)
                .setColor(2,Color.BLACK);
        ledRiver.setPattern(LEDRiver.Pattern.RUNNING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.reset();
        Thread.sleep(3000);
    }
}
