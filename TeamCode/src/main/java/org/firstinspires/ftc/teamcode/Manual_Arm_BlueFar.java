package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ARM Manual - Blue FarFarAway Hang", group = "Match")
public class Manual_Arm_BlueFar extends LinearOpMode {
    public String workingMode = "bluefar";
    public String robotType = RobotDataBase.defaultType;
    public boolean parkingCenter = true;
    public boolean autoMode = false;
    public boolean useCamera = false;
    public boolean debugMode = false;
    public boolean hangFarSide = true;

    public GetOurJobDone job = new GetOurJobDone();

    @Override
    public void runOpMode() throws InterruptedException {
        job.op = this;
        job.workingMode = workingMode;
        job.robotType = robotType;
        job.parkingCenter = parkingCenter;
        job.autoMode = autoMode;
        job.useCamera = useCamera;
        job.debugMode = debugMode;
        job.hangFarSide = hangFarSide;

        job.init();

        waitForStart();

        if (isStopRequested()) return;

        job.initAfterStart();

        while (opModeIsActive()) {
            job.runAfterStart();
            sleep(RobotDataBase.sleepMainThreadMilliSeconds);
        }

        job.stop();
    }
}

