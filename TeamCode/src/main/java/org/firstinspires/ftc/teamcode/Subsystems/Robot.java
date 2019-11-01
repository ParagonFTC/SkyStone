package org.firstinspires.ftc.teamcode.Subsystems;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "RobotLog";

    public MecanumDriveWrapper drive;
    public Intake intake;
    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;

    //private ExpansionHubEx hub1, hub2;

    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                }
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    public Robot(OpMode opMode) {
        subsystems = new ArrayList<>();

        //hub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        //hub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        try {
            drive = new MecanumDriveWrapper(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Mecanum Drive");
        }

        try {
            intake = new Intake(opMode.hardwareMap);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Intake");
        }

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);
        subsystems.add(drive);
        subsystems.add(intake);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        RobotLog.unregisterGlobalWarningSource(this);
    }
    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        List<String> globalWarnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                globalWarnings.add("Problem with" + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(globalWarnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
