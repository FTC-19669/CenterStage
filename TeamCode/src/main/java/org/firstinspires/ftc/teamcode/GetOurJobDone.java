package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.util.Size;
import java.util.Arrays;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class GetOurJobDone {
    public String workingMode = "rednear";
    public String robotType = "arm";
    public boolean parkingCenter = true;
    public RobotDataBase robotData = null;
    public boolean autoMode = false;
    public boolean useCamera = false;
    public boolean debugMode = false;
    public boolean hangFarSide = false;
    public LinearOpMode op;

    public double inchesOneSquare = 24;
    public DcMotor _fl, _fr, _rl, _rr;
    public DcMotor _slider, _swiper, _lifter, _transportation;
    public Servo _drone, _grip, _arm, _door, _swiperlifter, _lifterarm, _box;
    public CRServo _lifterhelper;
    public DistanceSensor _distanceGrip, _distanceSlider;
    public IMU _imu;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public int DESIRED_TAG_ID = 9;     // Choose the tag you want to approach or set to -1 for ANY tag.

    public boolean logMode = false;
    public ArrayList<String> logArray = new ArrayList<>();
    public boolean firstTime = true;
    public boolean pwmEnable = true;

    public ElapsedTime recentActionTime = new ElapsedTime();
    public boolean enablePad1Control = false;
    public double ratioPad2WheelSpeed = 0.0; //pad2 can control wheel at the ratio speed of pad1, 0 means pad2 stick can't wheels, 1 means same as pad1

    public double wheelTurnSpeed = 2.0;
    public double zoomRatio = 1.0;
    public int parkingPosition = 1;
    public int markPosition = 0; // left by default
    public boolean markSeen = false;
    public double xMark = 0;
    public double yMark = 0;

    public boolean prepareForManual = true;
    public boolean autoE2E = false;
    public int targetFrontLeft = 0, targetFrontRight = 0, targetRearLeft = 0, targetRearRight = 0;
    public String[][][] markDriveData;
    public int actionCount = 0;

    public ElapsedTime moreTimeToStart = new ElapsedTime();
    public ElapsedTime timeSinceStart = new ElapsedTime();
    public boolean stopPresetAction = false;
    public boolean cameraStreaming = true;
    public boolean hangReady = false;

    ThreadWheel threadWheel = null;
    ThreadArm threadArm = null;
    Thread tWheel = null;
    Thread tArm = null;

    public boolean distanceWheelOperationNow = false;
    public boolean imuWheelOperationNow = false;
    public boolean wheelSlider = true; // able to run with distanceslider
    public boolean recordXY = false;
    // positive value like 10.0 means robot has been wheel_right 10.0 inches
    // other wheel_forward need consider this
    public double deltaX = 0.0; //
    // positive value like 10.0 means robot has been wheel_forward 10.0 inches
    // other wheel_forward need consider this
    public double deltaY = 0.0;

    public void init() throws InterruptedException {
        if (useCamera) {
            //initAprilTag();
            initTfod();
            //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        _imu = op.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        _imu.initialize(new IMU.Parameters(orientationOnRobot));
        //_imu.resetYaw();

        if (robotType.equals("arm")) {
            robotData = new RobotDataArm(parkingCenter, workingMode, debugMode, hangFarSide);
        } else if (robotType.equals("armnew")) {
            robotData = new RobotDataArmNew(parkingCenter, workingMode, debugMode);
        } else {
            robotData = new RobotDataSwiper(parkingCenter, workingMode, debugMode);
        }

        if (workingMode.contains("rednear")) {
            markDriveData = robotData.markRedNear;
        } else if (workingMode.contains("redfar")) {
            markDriveData = robotData.markRedFar;
        } else if (workingMode.contains("bluenear")) {
            markDriveData = robotData.markBlueNear;
        } else if (workingMode.contains("bluefar")) {
            markDriveData = robotData.markBlueFar;
        }

        _fl = op.hardwareMap.get(DcMotor.class, "frontleft");
        _fr = op.hardwareMap.get(DcMotor.class, "frontright");
        _rl = op.hardwareMap.get(DcMotor.class, "rearleft");
        _rr = op.hardwareMap.get(DcMotor.class, "rearright");
        _slider = op.hardwareMap.get(DcMotor.class, "slider");
        _drone = op.hardwareMap.get(Servo.class, "drone");
        if (robotData.useDistanceSensor) {
            _distanceGrip = op.hardwareMap.get(DistanceSensor.class, "distancegrip");
            _distanceSlider = op.hardwareMap.get(DistanceSensor.class, "distanceslider");
        }

        if (robotData.type.startsWith("swiper")) {
            //_transportation = op.hardwareMap.get(DcMotor.class, "transportation");
            _swiper = op.hardwareMap.get(DcMotor.class, "swiper");
            _grip = op.hardwareMap.get(Servo.class, "grip");
            _swiperlifter = op.hardwareMap.get(Servo.class, "swiperlifter");
            _lifterarm = op.hardwareMap.get(Servo.class, "lifterarm");
        }

        if (robotData.type.startsWith("arm")) {
            _grip = op.hardwareMap.get(Servo.class, "grip");
            _arm = op.hardwareMap.get(Servo.class, "arm");
            _door = op.hardwareMap.get(Servo.class, "door");
            _box = op.hardwareMap.get(Servo.class, "box");
            _fl.setDirection(DcMotor.Direction.FORWARD);
            _rl.setDirection(DcMotor.Direction.FORWARD);
            _fr.setDirection(DcMotor.Direction.REVERSE);
            _rr.setDirection(DcMotor.Direction.REVERSE);
            _slider.setDirection(DcMotor.Direction.REVERSE);
        } else {
            // Reverse the right side motors
            _fl.setDirection(DcMotor.Direction.FORWARD);
            _rl.setDirection(DcMotor.Direction.FORWARD);
            _fr.setDirection(DcMotor.Direction.REVERSE);
            _rr.setDirection(DcMotor.Direction.REVERSE);
            _slider.setDirection(DcMotor.Direction.REVERSE);
        }

        if (robotData.useLifter) {
            _lifter = op.hardwareMap.get(DcMotor.class, "lifter");
            if (robotData.useFastLifter) {
                _lifterarm = op.hardwareMap.get(Servo.class, "lifterarm");
            }
            else {
                _lifterhelper = op.hardwareMap.get(CRServo.class, "lifterhelper");
                _lifterhelper.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        // RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
        _fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //_rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //_rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        op.telemetry.addData("status: ", "init completed, ready for press the start button");
        if (robotData.useIMU) {
            YawPitchRollAngles orientation = _imu.getRobotYawPitchRollAngles();
            op.telemetry.addData("Yaw (Z), Pitch (X), Roll (Y)", "%.2f Deg. (Heading), %.2f Deg, %.2f Deg",
                    orientation.getYaw(AngleUnit.DEGREES), orientation.getPitch(AngleUnit.DEGREES), orientation.getRoll(AngleUnit.DEGREES));
        }
        op.telemetry.update();
    }

    public void initAfterStart() throws InterruptedException {
        timeSinceStart.reset();

        if (!autoMode) {
            threadWheel = new ThreadWheel();
            threadArm = new ThreadArm();
            tWheel = new Thread(threadWheel);
            tArm = new Thread(threadArm);
            replayActionList(robotData.presetActionsInitAfterStartNonAutoMode);
        }
    }

    private final ElapsedTime lastMainActionTime = new ElapsedTime();
    private String lastMainActionName = "";

    public void runAfterStart() throws InterruptedException {
        if (firstTime) {
            firstTime = false;
            robotData.sliderInitialPosition = _slider.getCurrentPosition();
            targetFrontLeft = _fl.getCurrentPosition();
            targetFrontRight = _fr.getCurrentPosition();
            targetRearLeft = _rl.getCurrentPosition();
            targetRearRight = _rr.getCurrentPosition();
            if (autoMode) {
                if (workingMode.equals("rednear")) {
                    replayActionList(robotData.presetActionsRedNear);
                } else if (workingMode.equals("redfar")) {
                    replayActionList(robotData.presetActionsRedFar);
                } else if (workingMode.equals("bluenear")) {
                    replayActionList(robotData.presetActionsBlueNear);
                } else if (workingMode.equals("bluefar")) {
                    replayActionList(robotData.presetActionsBlueFar);
                } else if (workingMode.contains("purpleonly")) {
                    replayActionList(robotData.presetActionsDropPurpleOnly);
                }
            } else {
                tWheel.start();
                tArm.start();
            }
        }

        if (autoMode)
            return;

        boolean ignoreSame = lastMainActionTime.milliseconds() < robotData.minTimeOfTwoOperations;
        if (op.gamepad1.back) {
            if (ignoreSame && lastMainActionName.equals("back"))
                return;
            replayActionList(robotData.presetActionsPad1Back);
            lastMainActionName = "back";
            lastMainActionTime.reset();
            return;
        }
        if (distanceWheelOperationNow || imuWheelOperationNow) {
            return;
        }

        // enable for both controller
        double y=0, x=0, rx=0, denominator;
        double frontLeftPower, rearLeftPower, frontRightPower, rearRightPower, speedmultiplier;

        if (robotData.useNewManualDriveMethod) {
            // get y value from left_stick and x from right_stick
            y = op.gamepad1.left_stick_y;
            if (y == 0 || Math.abs(y) < Math.abs(op.gamepad1.left_stick_x)) {
                y = -op.gamepad1.left_stick_x;
            }
            if (op.gamepad1.left_bumper || op.gamepad1.right_bumper) {
                rx = -op.gamepad1.right_stick_x;
            }
            else {
                x = -op.gamepad1.right_stick_x;
                if (x == 0 || Math.abs(op.gamepad1.right_stick_y) > Math.abs(x)) {
                    x = -op.gamepad1.right_stick_y;
                }
            }
            x *= 0.5;
            y *= 0.5;
            rx *= 0.5;
        }
        else {
            y = op.gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
            x = -op.gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
            if (op.gamepad1.right_trigger > 0) {
                rx = -op.gamepad1.right_stick_x * 0.5;
            } else {
                rx = -op.gamepad1.right_stick_x * 0.5 * -1;
            }
        }

        //if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0) {
        //y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        //x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        //rx = -gamepad1.right_stick_x * 0.5;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        rearLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        rearRightPower = (y + x - rx) / denominator;
        speedmultiplier = -1;
        if (op.gamepad1.left_trigger > 0 && op.gamepad1.right_trigger > 0) {
            speedmultiplier = 0.2;
        }
        else if (op.gamepad1.left_trigger > 0) {
            speedmultiplier = -0.2;
        } else if (op.gamepad1.right_trigger > 0) {
            speedmultiplier = 1;
        }

        speedmultiplier *= robotData.fastWheelSpeedManual;

        if (op.gamepad1.left_bumper || op.gamepad1.left_stick_button || op.gamepad1.right_stick_button) {
            speedmultiplier *= 2;
        }

        if (distanceWheelOperationNow) {
            return;
        }
        _fl.setPower(frontLeftPower * speedmultiplier);
        _rl.setPower(rearLeftPower * speedmultiplier);
        _fr.setPower(frontRightPower * speedmultiplier);
        _rr.setPower(rearRightPower * speedmultiplier);
    /*
    if (frontLeftLastPower != frontLeftPower) {
        _fl.setPower(frontLeftPower * speedmultiplier);
        frontLeftLastPower = frontLeftPower;
    }
    if (rearLeftLastPower != rearLeftPower) {
        _rl.setPower(rearLeftPower * speedmultiplier);
        rearLeftLastPower = rearLeftPower;
    }
    if (frontRightLastPower != frontRightPower) {
        _fr.setPower(frontRightPower * speedmultiplier);
        frontRightLastPower = frontRightPower;
    }
    if (rearRightLastPower != rearRightPower) {
        _rr.setPower(rearRightPower * speedmultiplier);
        rearRightLastPower = rearRightPower;
    }
     */

    }

    public void stop() throws InterruptedException{
        if (!autoMode) {
            threadWheel.stop();
            threadArm.stop();
        }
        if (useCamera && cameraStreaming) {
            visionPortal.setProcessorEnabled(tfod, false);
            sleep(100);
            visionPortal.close();
        }
    }

    private void logAction(String s) {
        if (logMode) {
            logArray.add(s);
        } else {
            //return;
        }
        op.telemetry.addData("logAction", "count %d, action %s", actionCount, s);
        op.telemetry.addData("drone ", _drone.getPosition());
        if (robotData.type.startsWith("arm")) {
            op.telemetry.addData("grip ", _grip.getPosition());
            op.telemetry.addData("_arm ", _arm.getPosition());
            op.telemetry.addData("_box ", _box.getPosition());
            op.telemetry.addData("_door ", _door.getPosition());
        }

        op.telemetry.addData("markPosition", "value: %d,  x=%f y=%f", markPosition, xMark, yMark);
        op.telemetry.addData("running to", " %7d :%7d :%7d :%7d", targetFrontLeft, targetFrontRight, targetRearLeft, targetRearRight);
        op.telemetry.addData("currently at", " at %7d :%7d :%7d :%7d", _fl.getCurrentPosition(), _fr.getCurrentPosition(), _rl.getCurrentPosition(), _rr.getCurrentPosition());
        op.telemetry.addData("slider", " %7d", _slider.getCurrentPosition());
        if (robotData.useLifter) {
            op.telemetry.addData("lifter", " %7d", _lifter.getCurrentPosition());
            if (robotData.useFastLifter) {
                op.telemetry.addData("lifterarm", _lifterarm.getPosition());
            }
        }
        if (robotData.type.startsWith("swiper")) {
            op.telemetry.addData("grip ", _grip.getPosition());
            op.telemetry.addData("lifterarm", _lifterarm.getPosition());
            op.telemetry.addData("swiperlifter", _swiperlifter.getPosition());
        }
        if (robotData.useDistanceSensor) {
            //op.telemetry.addData("distanceGrip", String.format("%.01f cm / %.01f in", _distanceGrip.getDistance(DistanceUnit.CM), _distanceGrip.getDistance(DistanceUnit.INCH)));
            //op.telemetry.addData("distanceSlider", String.format("%.01f cm / %.01f in", _distanceSlider.getDistance(DistanceUnit.CM), _distanceSlider.getDistance(DistanceUnit.INCH)));
        }

        if (robotData.useIMU) {
            YawPitchRollAngles orientation = _imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = _imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            op.telemetry.addData("Yaw (Z), Pitch (X), Roll (Y)", "%.2f Deg. (Heading), %.2f Deg, %.2f Deg",
                    orientation.getYaw(AngleUnit.DEGREES), orientation.getPitch(AngleUnit.DEGREES), orientation.getRoll(AngleUnit.DEGREES));
            op.telemetry.addData("Yaw (Z) velocity, X, Y ", "%.2f Deg/Sec, %.2f, %.2f",
                    angularVelocity.zRotationRate, angularVelocity.xRotationRate, angularVelocity.yRotationRate);
        }

        op.telemetry.update();
    }

    private void resetToPresetPosition(int presetMode) {
        if (presetMode == 0) {
            logAction("Initial");
            //replayActions(presetActionsShoulderUp);
        }
    }

    public void resetServoPosition() {
    }


    private final double frontLeftLastPower = 0;
    private final double frontRightLastPower = 0;
    private final double rearLeftLastPower = 0;
    private final double rearRightLastPower = 0;
    private final ElapsedTime lastWheelActionTime = new ElapsedTime();
    private String lastWheelActionName = "";

    public void controlWheels() throws InterruptedException {
        boolean ignoreSame = lastWheelActionTime.milliseconds() < robotData.minTimeOfTwoOperations;
        if (op.gamepad1.a) {
            if (ignoreSame && lastWheelActionName.equals("a"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1AWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1AWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1A);
            lastWheelActionName = "a";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.b) {
            if (ignoreSame && lastWheelActionName.equals("b"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1BWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1BWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1B);
            lastWheelActionName = "b";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.x) {
            if (ignoreSame && lastWheelActionName.equals("x"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1XWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1XWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1X);
            lastWheelActionName = "x";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.y) {
            if (ignoreSame && lastWheelActionName.equals("y"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1YWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1YWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Y);
            lastWheelActionName = "y";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_left) {
            if (ignoreSame && lastWheelActionName.equals("left"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1LeftWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1LeftWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Left);
            lastWheelActionName = "left";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_right) {
            if (ignoreSame && lastWheelActionName.equals("right"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1RightWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1RightWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Right);
            lastWheelActionName = "right";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_up) {
            if (ignoreSame && lastWheelActionName.equals("up"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1UpWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1UpWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Up);
            lastWheelActionName = "up";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.dpad_down) {
            if (ignoreSame && lastWheelActionName.equals("down"))
                return;
            if (op.gamepad1.left_bumper)
                replayActionList(robotData.presetActionsPad1DownWithLeftBumper);
            else if (op.gamepad1.right_bumper)
                replayActionList(robotData.presetActionsPad1DownWithRightBumper);
            else
                replayActionList(robotData.presetActionsPad1Down);
            lastWheelActionName = "right";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.back) {
            if (ignoreSame && lastWheelActionName.equals("back"))
                return;
            replayActionList(robotData.presetActionsPad1Back);
            lastWheelActionName = "back";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.start) {
            if (ignoreSame && lastWheelActionName.equals("start"))
                return;
            replayActionList(robotData.presetActionsPad1Start);
            lastWheelActionName = "start";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.left_trigger > 0) {
            if (ignoreSame && lastWheelActionName.equals("left_trigger"))
                return;
            replayActionList(robotData.presetActionsPad1LeftTrigger);
            lastWheelActionName = "left_trigger";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.right_trigger > 0) {
            if (ignoreSame && lastWheelActionName.equals("right_trigger"))
                return;
            replayActionList(robotData.presetActionsPad1RightTrigger);
            lastWheelActionName = "right_trigger";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.left_bumper) {
            if (ignoreSame && lastWheelActionName.equals("left_bumper"))
                return;
            replayActionList(robotData.presetActionsPad1LeftBumper);
            lastWheelActionName = "left_bumper";
            lastWheelActionTime.reset();
            return;
        }
        if (op.gamepad1.right_bumper) {
            if (ignoreSame && lastWheelActionName.equals("right_bumper"))
                return;
            replayActionList(robotData.presetActionsPad1RightBumper);
            lastWheelActionName = "right_bumper";
            lastWheelActionTime.reset();
            return;
        }
        if (debugMode && op.gamepad1.options) {
            //setLogMode(!logMode);
            return;
        }
        if (debugMode && op.gamepad1.left_stick_button && op.gamepad1.right_stick_button) {
            //resetServoPosition();
        }
    }

    private final ElapsedTime lastArmActionTime = new ElapsedTime();
    private String lastArmActionName = "";

    private void controlArm() throws InterruptedException {
        boolean ignoreSame = lastArmActionTime.milliseconds() < robotData.minTimeOfTwoOperations;

        if (op.gamepad2.x) {
            if (ignoreSame && lastArmActionName.equals("x"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2XWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2XWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2XWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2X);
            }
            lastArmActionName = "x";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.a) {
            if (ignoreSame && lastArmActionName.equals("a"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2AWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2AWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2AWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2A);
            }
            lastArmActionName = "a";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.b) {
            if (ignoreSame && lastArmActionName.equals("b"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2BWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2BWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2BWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2B);
            }
            lastArmActionName = "b";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.y) {
            if (ignoreSame && lastArmActionName.equals("y"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2YWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2YWithRightBumper);
            } else if (op.gamepad2.left_stick_x < -0.5) {
                replayActionList(robotData.presetActionsPad2YWithLeftStickX);
            } else {
                replayActionList(robotData.presetActionsPad2Y);
            }
            lastArmActionName = "y";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_up) {
            if (ignoreSame && lastArmActionName.equals("up"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2UpWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2UpWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Up);
            }
            lastArmActionName = "up";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_down) {
            if (ignoreSame && lastArmActionName.equals("down"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2DownWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2DownWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Down);
            }
            lastArmActionName = "down";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_left) {
            if (ignoreSame && lastArmActionName.equals("left"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2LeftWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2LeftWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Left);
            }
            lastArmActionName = "left";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.dpad_right) {
            if (ignoreSame && lastArmActionName.equals("right"))
                return;
            if (op.gamepad2.left_bumper) {
                replayActionList(robotData.presetActionsPad2RightWithLeftBumper);
            } else if (op.gamepad2.right_bumper) {
                replayActionList(robotData.presetActionsPad2RightWithRightBumper);
            } else {
                replayActionList(robotData.presetActionsPad2Right);
            }
            lastArmActionName = "right";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.left_trigger > 0) {
            if (ignoreSame && lastArmActionName.equals("left_trigger"))
                return;
            replayActionList(robotData.presetActionsPad2LeftTrigger);
            lastArmActionName = "left_trigger";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.right_trigger > 0) {
            if (ignoreSame && lastArmActionName.equals("right_trigger"))
                return;
            replayActionList(robotData.presetActionsPad2RightTrigger);
            lastArmActionName = "right_trigger";
            lastArmActionTime.reset();
            return;
        }
        /*
        if (opMode.gamepad2.left_bumper) {
            if (ignoreSame && lastArmActionName.equals("left_bumper"))
                return;
            replayActionList(robotData.presetActionsPad2LeftBumper);;
            lastArmActionName = "left_bumper";
            lastArmActionTime.reset();
            return;
        }
        if (opMode.gamepad2.right_bumper) {
            if (ignoreSame && lastArmActionName.equals("right_bumper"))
                return;
            replayActionList(robotData.presetActionsPad2RightBumper);;
            lastArmActionName = "right_bumper";
            lastArmActionTime.reset();
            return;
        }
        */
        if (op.gamepad2.left_stick_button) {
            if (ignoreSame && lastArmActionName.equals("left_stick"))
                return;
            replayActionList(robotData.presetActionsPad2LeftStick);
            lastArmActionName = "left_stick";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.right_stick_button) {
            if (ignoreSame && lastArmActionName.equals("right_stick"))
                return;
            replayActionList(robotData.presetActionsPad2RightStick);
            lastArmActionName = "right_stick";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.back) {
            if (ignoreSame && lastArmActionName.equals("back"))
                return;
            replayActionList(robotData.presetActionsPad2Back);
            lastArmActionName = "back";
            lastArmActionTime.reset();
            return;
        }
        if (op.gamepad2.start) {
            if (ignoreSame && lastArmActionName.equals("start"))
                return;
            replayActionList(robotData.presetActionsPad2Start);
            lastArmActionName = "start";
            lastArmActionTime.reset();
        }

    }

    private void playOneStepAction(String actionName, boolean ignoreRecent) {
        if (ignoreRecent) {
            if (recentActionTime.milliseconds() < robotData.minTimeOfTwoOperations) {
                // too close to last action, ignore it
                return;
            } else {
                recentActionTime.reset();
            }
        }
        logAction(actionName);
    }

    private void setLogMode(boolean mode) {
        if (logMode == mode)
            return;
        if (mode) {
            logMode = true;
            logArray.clear();
            //System.out.println("Start log...");
            op.telemetry.addData("Start log...", logArray.size());
            op.telemetry.update();
        } else if (!mode) {
            op.telemetry.addData("Stop log...", logArray.size());
            logMode = false;

            // print out the moves / buttons pressed since log start;
            //System.out.println("Operations: " + logArray);
            for (int index = 0; index < logArray.size(); index++) {
                String s = Integer.toString(index);
                s += " ";
                s += logArray.get(index);
                op.telemetry.addData("Operations: ", s);
                op.telemetry.log().add(s);
            }
            //telemetry.addData("Operations: ", logArray);
            op.telemetry.update();
        }
    }

    public boolean replayAction(String s) throws InterruptedException {
        actionCount = actionCount + 1;
        //
        //"platform_left"
        //"platform_left@10"
        //"platform_left @ 10 @ 0.05"
        //
        String[] splitStrings = s.split("@", 6);
        for (int k = 0; k < splitStrings.length; k++) {
            splitStrings[k] = splitStrings[k].trim();
        }
        if (splitStrings.length == 0) {
            return true;
        }
        int repeatTimes = 1;

        if (splitStrings[0].contains("sleep")) {
            repeatTimes = Integer.parseInt(splitStrings[1]);
            waitElapsedTime(repeatTimes);
        }
        else if (splitStrings[0].startsWith("wheel")) {
            wheel(splitStrings);
        }
        else if (splitStrings[0].equals("imuturn")) {
            return imuTurn(splitStrings);
        }
        else if (splitStrings[0].equals("log")) {
            logAction("status ");
        }
        else if (splitStrings[0].equals("ai_getmarkposition")) {
            aiGetMarkPosition(splitStrings);
            if (markPosition != -1) // all
            {
                robotData.boxDropPosition = robotData.boxDropLowLeftPosition;
            }
        }
        else if (splitStrings[0].equals("ai_runtotag")) {
            aiRunToTag(splitStrings);
        }
        else if (splitStrings[0].equals("drive_markposition")) {
            driveMarkPosition();
        }
        //else if (splitStrings[0].startsWith("park_ai_position")) {
        //    parkAIPosition(splitStrings);
        //}
        else if (splitStrings[0].equals("nextstep")) {
            return nextStep(splitStrings);
        }
        else if (splitStrings[0].equals("drone")) {
            drone(splitStrings);
        }
        else if (splitStrings[0].equals("motor")) {
            motor(splitStrings);
        }
        else if (splitStrings[0].equals("grip")) {
            grip(splitStrings);
        }
        else if (splitStrings[0].equals("arm")) {
            arm(splitStrings);
        }
        else if (splitStrings[0].equals("lifterarm")) {
            lifterarm(splitStrings);
        }
        else if (splitStrings[0].equals("box")) {
            box(splitStrings);
        }
        else if (splitStrings[0].equals("swiperlifter")) {
            swiperlifter(splitStrings);
        }
        else if (splitStrings[0].startsWith("door")) {
            door(splitStrings);
        }
        else if (splitStrings[0].equals("swiper")) {
            swiper(splitStrings);
        }
        else if (splitStrings[0].equals("distancegrip")) {
            distanceGrip(splitStrings);
        }
        else if (splitStrings[0].equals("distanceslider")) {
            distanceSlider(splitStrings);
        }
        else if (splitStrings[0].startsWith("transportation")) {
            transportation(splitStrings);
        }
        else if (splitStrings[0].startsWith("slider")) {
            slider(splitStrings);
        }
        else if (splitStrings[0].equals("lifter")) {
            lifter(splitStrings);
        }
        else if (splitStrings[0].equals("lifterhelper")) {
            lifterhelper(splitStrings);
        }
        else if (splitStrings[0].equals("ai_doublecheckmarkposition")) {
            aiDoubleCheckMarkPosition(splitStrings);
        }
        else if (splitStrings[0].equals("balance")) {
            wheelBalance(splitStrings);
        }
        else if (splitStrings[0].equals("manual_wheel")) {
            manualWheel(splitStrings);
        }
        else if (splitStrings[0].equals("thread")) {
            replayInNewThread(splitStrings);
        }
        else if (splitStrings[0].equals("recordposition")) {
            recordPosition(splitStrings);
        }
        else if (splitStrings[0].equals("checktimepassedlessthan")) {
            return checkTimePassedLessThan(splitStrings);
        }
        else if (splitStrings[0].equals("checkdistanceanddrivetobackdrop")) {
            // not return here if want to continue the next operations like drop the pixels automatically
            return checkDistanceAndDriveToBackdrop(splitStrings);
        }
        else if (splitStrings[0].equals("exit")) {
            return false;
        }
        else if (splitStrings[0].equals("camera")) {
            cameraSetting(splitStrings);
        }
        else if (splitStrings[0].equals("ai_getmarkpositionbydistance")) {
            aiGetMarkPosition(splitStrings);
        }
        else if (splitStrings[0].equals("set")) {
            // not return here if want to continue the next operations like drop the pixels automatically
            setParameters(splitStrings);
        }
        else if (splitStrings[0].equals("check")) {
            return checkParameters(splitStrings);
        }
        else if (splitStrings[0].startsWith("position") && splitStrings.length >= 2) {
            position(splitStrings[0], splitStrings[1]);
        }
        else {
        }
        return true;
    }

    public void setParameters(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 3) {
            return;
        }
        if (splitStrings[1].equals("drivespeed")) {
            double speed = 1.0;
            if (splitStrings[2].equals("default")) {
                speed = 1.0;
            }
            else {
                speed = Double.parseDouble(splitStrings[2]);
            }
            robotData.fastWheelSpeedManual = speed;
        }
        else if (splitStrings[1].equals("scriptstopable")) {
            boolean stopable = false;
            if (splitStrings[2].equals("1") || splitStrings[2].equals("yes")) {
                stopable = true;
            }
            else if (splitStrings[2].equals("0") || splitStrings[2].equals("no")) {
                stopable = false;
            }
            robotData.scriptStopable = stopable;
        }
        else if (splitStrings[1].equals("armpower")) {
            if (splitStrings[2].equals("off")) {
                ServoController controller = _arm.getController();
                //if (controller.getPwmStatus() != ServoController.PwmStatus.DISABLED) {
                    controller.pwmDisable();
                //}
            }
            else if (splitStrings[2].equals("on")) {
                ServoController controller = _arm.getController();
                controller.pwmEnable();
            }
        }
        else if (splitStrings[1].equals("lifterarmpower")) {
            if (splitStrings[2].equals("off")) {
                ServoController controller = _lifterarm.getController();
                if (controller.getPwmStatus() != ServoController.PwmStatus.DISABLED) {
                    controller.pwmDisable();
                }
            }
            else if (splitStrings[2].equals("on")) {
                ServoController controller = _lifterarm.getController();
                controller.pwmEnable();
            }
        }
        else if (splitStrings[1].equals("hangready")) {
            if (splitStrings[2].equals("on")) {
                hangReady = true;
            }
            else if (splitStrings[2].equals("off")) {
                hangReady = false;
            }
        }
        else if (splitStrings[1].equals("wheelslider")) {
            if (splitStrings[2].equals("on")) {
                wheelSlider = true;
            }
            else if (splitStrings[2].equals("off")) {
                wheelSlider = false;
            }
        }
        else if (splitStrings[1].equals("imu")) {
            _imu.resetYaw();
        }
    }

    public boolean checkParameters(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 3) {
            return false;
        }

        if (splitStrings[1].equals("hangready")) {
            if (splitStrings[2].equals("on")) {
                return hangReady;
            }
            else if (splitStrings[2].equals("off")) {
                return !hangReady;
            }
        }
        else if (splitStrings[1].equals("imu")) {
            double targetYaw = Double.parseDouble(splitStrings[2]);
            // Get the current yaw angle from the IMU sensor
            double currentYaw = _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Calculate the error between the target and current yaw angles
            double error = targetYaw - currentYaw;
            // Check if the error is within the tolerance
            return Math.abs(error) < 10;
        }
        return false;
    }

    public boolean replayInNewThread(String[] splitStrings) throws InterruptedException {
        boolean result = true;
        if (splitStrings.length < 2) {
            return result;
        }
        String action = "";
        // reform the action string without the thread
        for (int i=1; i<splitStrings.length; i++) {
            String temp = i == 1 ? splitStrings[i] : "@"+splitStrings[i];
            action = action + temp;
            action = action + " ";
        }
        //op.telemetry.addData("action ", action);
        //op.telemetry.update();
        //sleep(2000);

        if (robotData.replayInNewThreadWhenUsingThreadKeyword) {
            ThreadReplay threadReplay = null;
            Thread tReplay = null;
            threadReplay = new ThreadReplay();
            threadReplay.action = action;
            tReplay = new Thread(threadReplay);
            tReplay.start();
        }
        else {
            replayAction(action);
        }
        return result;
    }


    public boolean checkTimePassedLessThan(String[] splitStrings) {
        boolean result = true;
        if (splitStrings.length < 2) {
            return result;
        }

        int expectedMilliSeconds = Integer.parseInt(splitStrings[1]);
        if (timeSinceStart.milliseconds() > expectedMilliSeconds) {
            result = false;
        }
        return result;
    }

    public boolean checkDistanceAndDriveToBackdrop(String[] splitStrings) throws InterruptedException {
        boolean result = true;
        if (splitStrings.length < 2) {
            return result;
        }

        double expectedDistanceToBackdrop = Double.parseDouble(splitStrings[1]);
        double distanceToBackdrop = getDistanceAverageByInch(_distanceSlider, 1);
        if (distanceToBackdrop > 72) {
            // can't detect one or maybe too far away from the target
            result = false;
        }
        else if (distanceToBackdrop > expectedDistanceToBackdrop) {
            String cmd = String.format("distanceslider @%s @1.0 @3000", splitStrings[1]);
            replayAction(cmd);
        }
        else {
            // already within the expected distance, no need to drive
        }
        return result;
    }

    public boolean checkGripAndDriveToBackdrop(String[] splitStrings) throws InterruptedException {
        boolean result = true;
        if (splitStrings.length < 2) {
            return result;
        }

        double expectedDistanceToBackdrop = Double.parseDouble(splitStrings[1]);
        double distanceToBackdrop = getDistanceAverageByInch(_distanceGrip, 1);
        double delta = distanceToBackdrop - expectedDistanceToBackdrop;
        if (delta > 48) {
            // can't detect one or maybe too far away from the target
            result = false;
        }
        else if (delta > 0.5) {
            String cmd = String.format("distancegrip @%s @0.2 @2000", splitStrings[1]);
            replayAction(cmd);
        }
        else if (delta < -0.5) {
            String cmd = String.format("distancegrip @%s @0.2 @2000", splitStrings[1]);
            replayAction(cmd);
        }
        else {
            // already within the expected distance, no need to drive
        }
        return result;
    }

    public void manualWheel(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length < 1)
            return;
        if (splitStrings[1].equals("enable")) {
            distanceWheelOperationNow = false;
        }
        else if (splitStrings[1].equals("disable")) {
            distanceWheelOperationNow = true;
        }
    }

    public void recordPosition(String[] splitStrings) throws InterruptedException {
        double position = 0;
        if (splitStrings.length < 1)
            return;
        String sCommand;
        if (splitStrings[1].equals("stop")) {
            op.telemetry.addData("position ", "deltaX: %f, deltaY: %f", deltaX, deltaY);
            op.telemetry.update();
            if (deltaX > 0) {
                // need return back
                sCommand = String.format("wheel_right @%6.7f @0.3 @nowait", Math.abs(deltaX));
            }
            else {
                sCommand = String.format("wheel_right @%6.7f @0.3 @nowait", Math.abs(deltaX));
            }
            replayAction(sCommand);
            if (deltaY > 0) {
                // need return back
                sCommand = String.format("wheel_back @%6.7f @0.3 @nowait", Math.abs(deltaY));
            }
            else {
                sCommand = String.format("wheel_forward @%6.7f @0.3 @nowait", Math.abs(deltaY));
            }
            replayAction(sCommand);
            deltaX = 0;
            deltaY = 0;
            recordXY = false;
        } else if (splitStrings[1].equals("start")) {
            recordXY = true;
            deltaX = 0;
            deltaY = 0;
        }
    }

    public void drone(String[] splitStrings) {
        double position = 0;
        if (splitStrings.length < 1)
            return;
        if (splitStrings[1].equals("close")) {
            _drone.setPosition(robotData.droneClosePosition);
            return;
        } else if (splitStrings[1].equals("default")) {
            _drone.setPosition(robotData.droneDefaultPosition);
            return;
        } else if (splitStrings[1].equals("open")) {
            _drone.setPosition(robotData.droneOpenPosition);
            return;
        } else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position")) {
            position = Double.parseDouble(splitStrings[2]);
            _drone.setPosition(position);
            return;
        } else {
            position = Double.parseDouble(splitStrings[1]);
        }
        double destPosition = _drone.getPosition() + position;
        op.telemetry.addData("drone ", "%f current: %f, dest: %f", position, _drone.getPosition(), destPosition);
        if (destPosition >= robotData.droneMinPosition && destPosition <= robotData.droneMaxPosition) {
            _drone.setPosition(destPosition);
            op.telemetry.addData("drone ", "after %f", _drone.getPosition());
        }
        op.telemetry.update();
    }

    public void grip(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("close")) {
            _grip.setPosition(robotData.gripClosePosition);
            return;
        } else if (splitStrings[1].equals("default")) {
            _grip.setPosition(robotData.gripDefaultPosition);
            return;
        } else if (splitStrings[1].equals("open") || splitStrings[1].equals("openlarge") || splitStrings[1].equals("large")) {
            _grip.setPosition(robotData.gripOpenLargePosition);
            return;
        } else if (splitStrings[1].equals("opensmall") || splitStrings[1].equals("small")) {
            _grip.setPosition(robotData.gripOpenSmallPosition);
            return;
        } else if (splitStrings[1].equals("verylarge")) {
            _grip.setPosition(robotData.gripOpenVeryLargePosition);
            return;
        }
        else if (splitStrings[1].equals("middle"))  {
                _grip.setPosition(robotData.gripOpenMiddlePosition);
                return;
        } else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position")) {
            position = Double.parseDouble(splitStrings[2]);
            _grip.setPosition(position);
            return;
        } else {
            position = Double.parseDouble(splitStrings[1]);
        }
        double destPosition = _grip.getPosition() + position;
        op.telemetry.addData("grip ", "%f current: %f, dest: %f", position, _grip.getPosition(), destPosition);
        if (destPosition >= robotData.gripMinPosition && destPosition <= robotData.gripMaxPosition) {
            _grip.setPosition(destPosition);
            op.telemetry.addData("grip ", "after %f", _grip.getPosition());
        }
        op.telemetry.update();
    }

    public void box(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("pickup")) {
            _box.setPosition(robotData.boxPickupPosition);
            return;
        }
        else if (splitStrings[1].equals("pixelslide")) {
            _box.setPosition(robotData.boxPixelSlidePosition);
            return;
        }
        else if (splitStrings[1].equals("drop")) {
            _box.setPosition(robotData.boxDropPosition);
            return;
        }
        else if (splitStrings[1].equals("droplow")) {
            _box.setPosition(robotData.boxDropLowPosition);
            return;
        }
        else if (splitStrings[1].equals("default")) {
            _box.setPosition(robotData.boxDefaultPosition);
            return;
        }
        else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position") ) {
            position = Double.parseDouble(splitStrings[2]);
            _box.setPosition(position);
            return;
        }
        else {
            position = Double.parseDouble(splitStrings[1]);
        }
        double destPosition = _box.getPosition() + position;
        op.telemetry.addData("box ", "%f current: %f, dest: %f", position, _box.getPosition(), destPosition);
        if (destPosition >= robotData.boxMinPosition && destPosition <= robotData.boxMaxPosition) {
            _box.setPosition(destPosition);
            op.telemetry.addData("box ", "after %f", _box.getPosition());
        }
        op.telemetry.update();
    }

    public void swiper(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double power = Double.parseDouble(splitStrings[1]);
        _swiper.setPower(power);
    }

    public void swiperlifter(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("up")) {
            _swiperlifter.setPosition(robotData.swiperlifterUpPosition);
            return;
        }
        else if (splitStrings[1].equals("down")) {
            _swiperlifter.setPosition(robotData.swiperlifterDownPosition);
            return;
        }
        else if (splitStrings[1].equals("default")) {
            _swiperlifter.setPosition(robotData.swiperlifterDefaultPosition);
            return;
        }
        else if (splitStrings[1].equals("high5")) {
            _swiperlifter.setPosition(robotData.swiperlifterHigh5Position);
            return;
        }
        else if (splitStrings[1].equals("high4")) {
            _swiperlifter.setPosition(robotData.swiperlifterHigh4Position);
            return;
        }
        else if (splitStrings[1].equals("high3")) {
            _swiperlifter.setPosition(robotData.swiperlifterHigh3Position);
            return;
        }
        else if (splitStrings[1].equals("high2")) {
            _swiperlifter.setPosition(robotData.swiperlifterHigh2Position);
            return;
        }
        else if (splitStrings[1].equals("high1")) {
            _swiperlifter.setPosition(robotData.swiperlifterHigh1Position);
            return;
        }
        else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position") ) {
            position = Double.parseDouble(splitStrings[2]);
            _swiperlifter.setPosition(position);
            return;
        }
        else {
            position = Double.parseDouble(splitStrings[1]);
        }

        double destPosition = _swiperlifter.getPosition() + position;
        op.telemetry.addData("swiperlifter ", "%f current: %f, dest: %f", position, _swiperlifter.getPosition(), destPosition);
        if (destPosition >= robotData.swiperlifterMinPosition && destPosition <= robotData.swiperlifterMaxPosition) {
            _swiperlifter.setPosition(destPosition);
            op.telemetry.addData("swiperlifter ", "after %f", _swiperlifter.getPosition());
        }
        op.telemetry.update();
    }

    public void motor(String[] splitStrings) {
        if (splitStrings.length < 3)
            return;
        double power = 0;
        DcMotor motor = _fl;
        if (splitStrings[1].equals("frontleft")) {
            motor = _fl;
        }
        else if (splitStrings[1].equals("frontright")) {
            motor = _fr;
        }
        else if (splitStrings[1].equals("rearleft")) {
            motor = _rl;
        }
        else if (splitStrings[1].equals("rearright")) {
            motor = _rr;
        }
        else if (splitStrings[1].equals("slider")) {
            motor = _slider;
        }
        power = Double.parseDouble(splitStrings[2]);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    public void transportation(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double power = Double.parseDouble(splitStrings[1]);
        _transportation.setPower(power);
    }

    public void slider(String[] splitStrings) throws InterruptedException {
        // make sure arm is not too close to the slider, otherwise it will collide with the box
        if (splitStrings.length < 1)
            return;
        int curPosition = _slider.getCurrentPosition();
        int destPosition = curPosition;
        if (splitStrings[1].equals("up")) {
            if ((curPosition + robotData.sliderSmallStepSize) >= (robotData.sliderInitialPosition + robotData.sliderMaxPosition))
                destPosition = robotData.sliderInitialPosition + robotData.sliderMaxPosition;
            else if ((curPosition + robotData.sliderSmallStepSize) <= (robotData.sliderInitialPosition + robotData.sliderFirstRowPosition))
                destPosition = robotData.sliderInitialPosition + robotData.sliderFirstRowPosition;
            else
                destPosition = curPosition + robotData.sliderSmallStepSize;
        }
        else if (splitStrings[1].equals("upbig")) {
            if ((curPosition + robotData.sliderStepSize) >= (robotData.sliderInitialPosition + robotData.sliderMaxPosition))
                destPosition = robotData.sliderInitialPosition + robotData.sliderMaxPosition;
            else if ((curPosition + robotData.sliderStepSize) <= (robotData.sliderInitialPosition + robotData.sliderFirstRowPosition))
                destPosition = robotData.sliderInitialPosition + robotData.sliderFirstRowPosition;
            else
                destPosition = curPosition + robotData.sliderStepSize;
        }
        else if (splitStrings[1].equals("down")) {
            if ((curPosition - robotData.sliderDownStepSize) <= (robotData.sliderInitialPosition + robotData.sliderMinPosition))
                destPosition = robotData.sliderInitialPosition + robotData.sliderMinPosition;
            else
                destPosition = curPosition - robotData.sliderDownStepSize;
        }
        else if (splitStrings[1].equals("min")) {
            destPosition = robotData.sliderInitialPosition;
        }
        else if (splitStrings[1].equals("max")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderMaxPosition;
        }
        else if (splitStrings[1].equals("high")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderHighPosition;
        }
        else if (splitStrings[1].equals("row0")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow0Position;
        }
        else if (splitStrings[1].equals("row1")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow1Position;
        }
        else if (splitStrings[1].equals("row1far")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow1FarPosition;
        }
        else if (splitStrings[1].equals("row2")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow2Position;
        }
        else if (splitStrings[1].equals("row3")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow3Position;
        }
        else if (splitStrings[1].equals("row4")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow4Position;
        }
        else if (splitStrings[1].equals("row5")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderRow5Position;
        }
        else if (splitStrings[1].equals("line1")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderLine1Position;
        }
        else if (splitStrings[1].equals("line2")) {
            destPosition = robotData.sliderInitialPosition + robotData.sliderLine2Position;
        }
        else if (splitStrings[1].equals("position")) {
            destPosition = Integer.parseInt(splitStrings[2]);
        }
        else {
            double power = Double.parseDouble(splitStrings[1]);
            _slider.setPower(power);
            return;
        }

        int timeToRun = 1000;
        if (splitStrings.length >= 3) {
            timeToRun = Integer.parseInt(splitStrings[2]);
        }
        if (destPosition != curPosition) {
            double power = robotData.sliderDefaultPower;
            if (splitStrings.length >=3) {
                power = Double.parseDouble(splitStrings[2]);
            }
            motorRunToPosition(_slider, destPosition, power, timeToRun, robotData.sliderUseSoftStop);
            if (splitStrings[1].equals("min")) {
                robotData.sliderInitialPosition = _slider.getCurrentPosition();
            }
        }
    }

    public void lifter(String[] splitStrings) throws InterruptedException {
        // lifter @1.0 @3000 // lifter run at 1.0 power for 3000 milliseconds
        // lifter @-1.0 @3000 // lifter run at 1.0 power for 3000 milliseconds
        // lifter @0.5  // nonstop
        // lifter @0.6 @percent @25
        double power = Double.parseDouble(splitStrings[1]);
        if (splitStrings.length == 2) {
            _lifter.setPower(power);
        }
        else if (splitStrings.length == 3) {
            _lifter.setPower(power);

            int rotationTime = Integer.parseInt(splitStrings[3]);
            waitElapsedTime(rotationTime);

            // stop power
            _lifter.setPower(0);
        }
        else if (splitStrings.length >3 && splitStrings[2].equals("percent")) {
            double destPositionPercent = Double.parseDouble(splitStrings[3]);
            int destPosition = (int) (destPositionPercent * robotData.lifterMaxPosition / 100);
            motorRunToPosition(_lifter, destPosition, power, 5000, false);
        }
    }

    public void lifterhelper(String[] splitStrings) throws InterruptedException {
        // lifterhelper @0.5 //nonstop
        // lifterhelper @0.5 @3000 // run at 0.5 power for 3000 milliseconds
        // lifterhelper @-0.8 @3000 // run at -0.8 power for 3000 milliseconds
        if (splitStrings.length < 2)
            return;

        double power = Double.parseDouble(splitStrings[1]);
        if (splitStrings.length == 2) {
            _lifterhelper.setPower(power);
        }
        else if (splitStrings.length == 3) {
            _lifterhelper.setPower(power);

            int rotationTime = Integer.parseInt(splitStrings[2]);
            waitElapsedTime(rotationTime);

            // stop power
            _lifterhelper.setPower(0);
        }
    }

    private void arm(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("up")) {
            _arm.setPosition(robotData.armUpPosition);
            return;
        }
        else if (splitStrings[1].equals("down")) {
            _arm.setPosition(robotData.armDownPosition);
            return;
        }
        else if (splitStrings[1].equals("default")) {
            _arm.setPosition(robotData.armDefaultPosition);
            return;
        }
        else if (splitStrings[1].equals("vertical")) {
            _arm.setPosition(robotData.armVerticalPosition);
            return;
        }
        else if (splitStrings[1].equals("moveready")) {
            _arm.setPosition(robotData.armMoveReadyPosition);
            return;
        }
        else if (splitStrings[1].equals("high5")) {
            _arm.setPosition(robotData.armHigh5Position);
            return;
        }
        else if (splitStrings[1].equals("high4")) {
            _arm.setPosition(robotData.armHigh4Position);
            return;
        }
        else if (splitStrings[1].equals("high3")) {
            _arm.setPosition(robotData.armHigh3Position);
            return;
        }
        else if (splitStrings[1].equals("high2")) {
            _arm.setPosition(robotData.armHigh2Position);
            return;
        }
        else if (splitStrings[1].equals("high1")) {
            _arm.setPosition(robotData.armHigh1Position);
            return;
        }
        else if (splitStrings[1].equals("firstrow")) {
            _arm.setPosition(robotData.armFirstRowPosition);
            return;
        }
        else if (splitStrings[1].equals("secondrow")) {
            _arm.setPosition(robotData.armSecondRowPosition);
            return;
        }
        else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position") ) {
            position = Double.parseDouble(splitStrings[2]);
            _arm.setPosition(position);
            return;
        }
        else {
            position = Double.parseDouble(splitStrings[1]);
        }

        double destPosition = _arm.getPosition() + position;
        op.telemetry.addData("arm ", "%f current: %f, dest: %f", position, _arm.getPosition(), destPosition);
        if (destPosition >= robotData.armMinPosition && destPosition <= robotData.armMaxPosition) {
            _arm.setPosition(destPosition);
            op.telemetry.addData("arm ", "after %f", _arm.getPosition());
        }
        op.telemetry.update();
    }

    private void lifterarm(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("up")) {
            _lifterarm.setPosition(robotData.lifterarmUpPosition);
            return;
        }
        else if (splitStrings[1].equals("down")) {
            _lifterarm.setPosition(robotData.lifterarmDownPosition);
            return;
        }
        else if (splitStrings[1].equals("default")) {
            _lifterarm.setPosition(robotData.lifterarmDefaultPosition);
            return;
        }
        else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position") ) {
            position = Double.parseDouble(splitStrings[2]);
            _lifterarm.setPosition(position);
            return;
        }
        else {
            position = Double.parseDouble(splitStrings[1]);
        }

        double destPosition = _lifterarm.getPosition() + position;
        op.telemetry.addData("lifterarm ", "%f current: %f, dest: %f", position, _lifterarm.getPosition(), destPosition);
        if (destPosition >= robotData.lifterarmMinPosition && destPosition <= robotData.lifterarmMaxPosition) {
            _lifterarm.setPosition(destPosition);
            op.telemetry.addData("lifterarm ", "after %f", _lifterarm.getPosition());
        }
        op.telemetry.update();
    }

    private void door(String[] splitStrings) {
        if (splitStrings.length < 1)
            return;
        double position = 0;
        if (splitStrings[1].equals("open")) {
            _door.setPosition(robotData.doorOpenPosition);
            //logAction("dooropen");
            return;
        }
        else if (splitStrings[1].equals("openone")) {
            _door.setPosition(robotData.doorOpenOnePosition);
            //logAction("doorclose");
            return;
        }
        else if (splitStrings[1].equals("close")) {
            _door.setPosition(robotData.doorClosePosition);
            //logAction("doorclose");
            return;
        }
        else if (splitStrings[1].equals("default")) {
            _door.setPosition(robotData.doorDefaultPosition);
            //logAction("doordefault");
            return;
        }
        else if (splitStrings[1].equals("pos") || splitStrings[1].equals("position") ) {
            position = Double.parseDouble(splitStrings[2]);
            _door.setPosition(position);
            return;
        }
        else {
            position = Double.parseDouble(splitStrings[1]);
        }
        double destPosition = _door.getPosition() + position;
        op.telemetry.addData("door ", "%f current: %f, dest: %f", position, _door.getPosition(), destPosition);
        if (destPosition >= robotData.doorMinPosition && destPosition <= robotData.doorMaxPosition) {
            _door.setPosition(destPosition);
            op.telemetry.addData("door ", "after %f", _door.getPosition());
        }
        op.telemetry.update();
    }

    public void motorRunToPosition(DcMotor motor, int destPosition, double powerIn, int maxWaitMilliSeconds, boolean useSoftStop) throws InterruptedException {
        if (useSoftStop) {
            motorRunToPositionWithSoftStop(motor, destPosition, powerIn, maxWaitMilliSeconds);
            return;
        }

        double power = powerIn;
        if (motor.getCurrentPosition() < destPosition) {
            power = 0 - powerIn;
        }
        ElapsedTime     motorRunTime = new ElapsedTime();

        motor.setTargetPosition(destPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while ((motorRunTime.milliseconds() < maxWaitMilliSeconds) && motor.isBusy()) {
            String s = String.format("motorRunToPosition %d", destPosition);
            logAction(s);
            sleep(1);
        }
        // Stop all motion;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public void motorRunToPositionWithSoftStop(DcMotor motor, int destPosition, double powerIn, int maxWaitMilliSeconds) throws InterruptedException {
        double power = powerIn;
        if (motor.getCurrentPosition() < destPosition) {
            power = 0 - powerIn;
        }
        ElapsedTime     motorRunTime = new ElapsedTime();
        double thresholdDistance = 100;
        double thresholdPower = 0.2;
        double softStopPower = 0.0;
        boolean pidErrorAdjust = false;
        // PID error variables
        double kP = 0.1;
        double kI = 0.0;
        double kD = 0.0;
        double lastError = 0.0;
        double totalError = 0.0;

        motor.setTargetPosition(destPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while ((motorRunTime.milliseconds() < maxWaitMilliSeconds) && motor.isBusy()) {

            int curPos = motor.getCurrentPosition();

            double distanceRemaining = destPosition - curPos;
            double maxDistanceRemaining = Math.abs(distanceRemaining);

            if (maxDistanceRemaining <= thresholdDistance) {
                softStopPower = power * (maxDistanceRemaining / thresholdDistance);
                // Apply the soft stop power
                if (softStopPower < thresholdPower) {
                    softStopPower = thresholdPower;
                }
                motor.setPower(power * softStopPower);
            }
            else {
                if (pidErrorAdjust) {
                    // calc the PID error and adjust the power
                    double error1 = distanceRemaining;
                    totalError = error1;
                    double powerAdjustment1 = kP * error1 + kI * totalError + kD * (error1 - lastError);
                    lastError = totalError;
                    motor.setPower(power - powerAdjustment1);
                }
            }
            String s = String.format("runToPositionWithSoftStop %d", destPosition);
            logAction(s);
            // sleep for a short period to allow the motor to move
            sleep(1);
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Stop all motion;
        motor.setPower(0);
    }

    public void distanceGrip(String[] splitStrings) throws InterruptedException {
        distanceWheelOperationNow = true;
        runToPositionWithDistanceSensor(_distanceGrip, -1.0, splitStrings);
        distanceWheelOperationNow = false;
    }

    public void distanceSlider(String[] splitStrings) throws InterruptedException {
        distanceWheelOperationNow = true;
        runToPositionWithDistanceSensor(_distanceSlider, 1.0, splitStrings);
        distanceWheelOperationNow = false;
    }

    public void runToPositionWithDistanceSensor(DistanceSensor distanceSensor, double dForward,
                                                String[] splitStrings) throws InterruptedException {
        double targetDistanceInch = Double.parseDouble(splitStrings[1]);
        double power = Double.parseDouble(splitStrings[2]);
        double thresholdPower = 0.1;
        double thresholdPower2 = 0.2;
        double thresholdDistance = 4; //5.0;
        double curDistance = getDistanceAverageByInch(distanceSensor, 1);
        int maxTime = 3000;
        if (splitStrings.length >= 4) {
            maxTime = Integer.parseInt(splitStrings[3]);
        }
        double delta = curDistance - targetDistanceInch;
        if (curDistance <= thresholdDistance || delta <= thresholdDistance) {
            power = power > thresholdPower ? thresholdPower : power;
        }
        else if (curDistance <= (thresholdDistance * 2) || delta <= (thresholdDistance * 2)) {
            power = power > thresholdPower2 ? thresholdPower2 : power;
        }
        ElapsedTime motorRunTime = new ElapsedTime();

        _fl.setPower(power * dForward);
        _fr.setPower(power * dForward);
        _rl.setPower(power * dForward);
        _rr.setPower(power * dForward);

        do {
            curDistance = getDistanceAverageByInch(distanceSensor, 1);
            if (curDistance > 100)
                continue;
            delta = curDistance - targetDistanceInch;
            if (delta <= 0) {
                break;
            }
            else if (curDistance <= thresholdDistance || delta <= thresholdDistance) {
                if (power > thresholdPower) {
                    power = thresholdPower;
                    _fl.setPower(power * dForward);
                    _fr.setPower(power * dForward);
                    _rl.setPower(power * dForward);
                    _rr.setPower(power * dForward);
                }
            }
            else if (curDistance <= (thresholdDistance * 2) || delta <= (thresholdDistance * 2)) {
                if (power > thresholdPower2) {
                    power = thresholdPower2;
                    _fl.setPower(power * dForward);
                    _fr.setPower(power * dForward);
                    _rl.setPower(power * dForward);
                    _rr.setPower(power * dForward);
                }
            }
            sleep(1);
        } while ((motorRunTime.milliseconds() < maxTime) && wheelSlider);

        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);
    }

    public boolean replayActionList(String[] list) throws InterruptedException {
        for (int i = 0; i < list.length; i++) {
            // stop the script when one of the keys are pressed
            if (robotData.scriptStopable && (op.gamepad1.left_bumper  || op.gamepad1.right_bumper || op.gamepad2.left_bumper || op.gamepad2.right_bumper)) {
                robotData.scriptStopable = false;
                return false;
            }
            if (robotData.debugReplayActionListWithSleep) {
                op.telemetry.addData("replay ", "no.%d %s", i, list[i]);
                op.telemetry.update();
                sleep(2000);
            }
            if (!replayAction(list[i])) {
                return false;
            }
        }
        return true;
    }

    String[] getStringListByName(String name) {
        if (name.equals("presetActionsStep2_left")) {
            return robotData.presetActionsStep2_left;
        }
        else if (name.equals("presetActionsStep2_right")) {
            return robotData.presetActionsStep2_right;
        }
        else {
            return robotData.presetActionsDefault;
        }
    }

    boolean nextStep(String[] splitStrings) throws InterruptedException {
        String[] arrlistActions;
        String direction = "left";
        if (splitStrings[1].equals("presetActionsGetWhitePixels")) {
            return replayActionList(robotData.presetActionsGetWhitePixels);
        }
        else if (splitStrings[1].equals("presetActionsGetOnePixelFromStack")) {
            return replayActionList(robotData.presetActionsGetOnePixelFromStack);
        }
        else if (splitStrings[1].equals("presetActionsGetTwoPixelFromStack")) {
            return replayActionList(robotData.presetActionsGetTwoPixelFromStack);
        }
        else if (splitStrings[1].equals("presetActionsGetAndDropPixels")) {
            return replayActionList(robotData.presetActionsGetAndDropPixels);
        }
        else if (splitStrings[1].equals("mark0")) {
            return replayActionList(markDriveData[markPosition][0]);
        }
        else if (splitStrings[1].equals("mark1")) {
            return replayActionList(markDriveData[markPosition][1]);
        }
        else if (splitStrings[1].equals("mark2")) {
            return replayActionList(markDriveData[markPosition][2]);
        }
        else if (splitStrings[1].equals("mark3")) {
            return replayActionList(markDriveData[markPosition][3]);
        }
        else if (splitStrings[1].equals("mark4")) {
            return replayActionList(markDriveData[markPosition][4]);
        }
        else if (splitStrings[1].equals("mark5")) {
            return replayActionList(markDriveData[markPosition][5]);
        }
        else if (splitStrings[1].equals("mark6")) {
            return replayActionList(markDriveData[markPosition][6]);
        }
        else if (splitStrings[1].equals("mark7")) {
            return replayActionList(markDriveData[markPosition][7]);
        }
        else if (splitStrings[1].equals("markRedLeftIndex1")) {
            return replayActionList(robotData.markRedLeftIndex1);
        }
        else if (splitStrings[1].equals("markRedFarRightIndex1")) {
            return replayActionList(robotData.markRedFarRightIndex1);
        }
        else if (splitStrings[1].equals("markRedFarRightIndex5")) {
            return replayActionList(robotData.markRedFarRightIndex5);
        }
        else if (splitStrings[1].equals("markBlueRightIndex1")) {
            return replayActionList(robotData.markBlueRightIndex1);
        }
        else if (splitStrings[1].equals("presetActionsRedNear")) {
            return replayActionList(robotData.presetActionsRedNear);
        }
        else {
            /*
            if (workingMode.equals("redleft") || workingMode.equals("blue_left")) {
                direction = "_left";
            } else {
                direction = "_right";
            }
            String actions = splitStrings[1] + direction;
            arrlistActions = getStringListByName(actions);
            replayActionList(arrlistActions);
            */
        }
        return true;
    }

    public void position(String target, String sPosition) {
        double targetPosition = Double.parseDouble(sPosition);
        if (targetPosition < 0 || targetPosition > 1)
            return;
        if (target.equals("position_platform")) {
            //_platform.setPosition(targetPosition);
        }
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 512 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265);

    public void wheel(String[] splitStrings) throws InterruptedException {
        String direction = splitStrings[0];
        ElapsedTime wheelRunTime = new ElapsedTime();
        double timeoutMilliSeconds = 10000;
        boolean waitForAllWheels = true;
        double distanceInches = 0;
        distanceInches = Double.parseDouble(splitStrings[1]);
        boolean useIMUKeepStraight = false;
        double targetYaw = 0;
        double minPower = -1;
        double maxPower = 1;

        if (distanceInches == 0) {
            return;
        }

        if (direction.equals("wheel_leftorright")) {
            if (distanceInches < 0) {
                direction = "wheel_left";
                distanceInches = 0 - distanceInches;
            }
            else if (distanceInches > 0) {
                direction = "wheel_right";
            }
        }

        double speed = Double.parseDouble(splitStrings[2]);
        //minPower = 0 - Math.abs(speed);
        //maxPower = Math.abs(speed);

        // go through other non-fixed parameters
        for (int index = 3; index < splitStrings.length; index++) {
            if (splitStrings[index].equals("nowait")) {
                waitForAllWheels = false;
            }
            else if (splitStrings[index].equals("yaw90")) {
                useIMUKeepStraight = true;
                targetYaw = 90;
                waitForAllWheels = false;
            }
            else if (splitStrings[index].equals("yaw-90")) {
                useIMUKeepStraight = true;
                targetYaw = -90;
                waitForAllWheels = false;
            }
            else if (splitStrings[index].equals("yaw0")) {
                useIMUKeepStraight = true;
                targetYaw = 0;
                waitForAllWheels = false;
            }
            else if (splitStrings[index].startsWith("yaw")) {
                // get the string after the yaw, to be done later
                useIMUKeepStraight = true;
                String sValue = splitStrings[index].substring(3);
                targetYaw = Double.parseDouble(sValue);
                waitForAllWheels = false;
            }
        }

        if (useIMUKeepStraight && !robotData.useIMU) {
            useIMUKeepStraight = false;
        }

        boolean useTotalSteps = false;
        boolean pidErrorAdjust = false;
        int newFrontLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newRearLeftTarget = 0;
        int newRearRightTarget = 0;

        if (direction.contains("same"))
            useTotalSteps = true;
        // reset the timeout time and start motion.
        wheelRunTime.reset();
        double frontLeftPower = 1;
        double frontRightPower = 1;
        double rearLeftPower = 1;
        double rearRightPower = 1;

        if (deltaY != 0 && !recordXY) {
            if (direction.equals("wheel_forward")) {
                distanceInches -= deltaY; // had run deltaY, this time no need to run this far
                deltaY = 0;
                if (distanceInches < 0) {
                    direction = "wheel_back";
                    distanceInches = 0 - distanceInches;
                }
            } else if (direction.equals("wheel_back")) {
                distanceInches += deltaY;
                deltaY = 0;
                if (distanceInches < 0) {
                    direction = "wheel_forward";
                    distanceInches = 0 - distanceInches;
                }
            }
        }
        if (deltaX != 0 && !recordXY) {
            if (direction.equals("wheel_right")) {
                distanceInches -= deltaX;
                deltaX = 0;
                if (distanceInches < 0) {
                    direction = "wheel_left";
                    distanceInches = 0 - distanceInches;
                }
            } else if (direction.equals("wheel_left")) {
                distanceInches += deltaX;
                deltaX = 0;
                if (distanceInches < 0) {
                    direction = "wheel_right";
                    distanceInches = 0 - distanceInches;
                }
            }
        }

        if (distanceInches == 0) {
            return;
        }

        if (direction.equals("wheel_forward") ) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft +=  - (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight +=  - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 1;
            frontRightPower = -1;
            rearLeftPower = 1;
            rearRightPower = -1;

            deltaY += recordXY ? distanceInches : 0;
        }
        else if (direction.equals("wheel_back") ) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft +=  (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight +=  (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 1;
            frontRightPower = 1;
            rearLeftPower = 1;
            rearRightPower = 1;

            deltaY -= recordXY ? distanceInches : 0;
        }
        else if (direction.equals("wheel_turn_left") ) {
            distanceInches = (distanceInches / 90) * robotData.distanceTurn90Degree;
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft +=  (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight +=  - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = -1;
            frontRightPower = 1;
            rearLeftPower = -1;
            rearRightPower = 1;
        }
        else if (direction.equals("wheel_turn_right") ) {
            distanceInches = (distanceInches / 90) * robotData.distanceTurn90Degree;
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft +=  - (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight +=  (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 1;
            frontRightPower = -1;
            rearLeftPower = 1;
            rearRightPower = -1;
        }
        else if (direction.equals("wheel_left") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft +=  (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += - (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight +=  (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 1;
            frontRightPower = -1;
            rearLeftPower = -1;
            rearRightPower = 1;

            deltaX -= recordXY ? distanceInches : 0;
        }
        else if (direction.equals("wheel_right") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            targetFrontLeft += - (int)(distanceInches * COUNTS_PER_INCH);
            targetFrontRight += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearLeft += (int)(distanceInches * COUNTS_PER_INCH);
            targetRearRight += - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = -1;
            frontRightPower = 1;
            rearLeftPower = 1;
            rearRightPower = -1;

            deltaX += recordXY ? distanceInches : 0;
        }

        if (useTotalSteps) {
            _fl.setTargetPosition(targetFrontLeft);
            _fr.setTargetPosition(targetFrontRight);
            _rl.setTargetPosition(targetRearLeft);
            _rr.setTargetPosition(targetRearRight);
        }
        else {
            _fl.setTargetPosition(newFrontLeftTarget);
            _fr.setTargetPosition(newFrontRightTarget);
            _rl.setTargetPosition(newRearLeftTarget);
            _rr.setTargetPosition(newRearRightTarget);
        }

        // Turn On RUN_TO_POSITION
        _fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _fl.setPower(frontLeftPower * speed);
        _fr.setPower(frontRightPower * speed);
        _rl.setPower(rearLeftPower * speed);
        _rr.setPower(rearRightPower * speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        double thresholdDistance = 100;
        double thresholdPower = 0.2;

        // PID error variables
        double kP = 0.01; // 0.1
        double kI = 0.01; //0.01;
        double kD = 0.001; //0.001;
        double lastError = 0.0;
        double totalError = 0.0;
        double softStopPower = 0.0;
        double loopTime = 0.05;
        double integral = 0;
        double derivative = 0;
        double leftPower = 0;
        double rightPower = 0;
        double leftPower2 = 0;
        double rightPower2 = 0;
        double leftPower3 = 0;
        double rightPower3 = 0;
        double correction = 0;

        // Get the current yaw angle from the IMU sensor
        double currentYaw = useIMUKeepStraight ? _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : 0;
        // Calculate the error between the target and current yaw angles
        double error = (targetYaw - currentYaw) / RobotDataBase.yawErrorRatioBase;
        double baseError = Math.abs(error) >= RobotDataBase.yawErrorBigValue ? Math.abs(error) : RobotDataBase.yawErrorBigValue;

        if (!useTotalSteps) {
            while (wheelRunTime.milliseconds() < timeoutMilliSeconds) {

                if (!waitForAllWheels) {
                    if (_fl.isBusy() && _fr.isBusy() && _rl.isBusy() && _rr.isBusy()) {
                        // all wheels are busy, need wait
                    }
                    else {
                        // one of the wheels has reached destination, only good for short distance and low power speed
                        break;
                    }
                }
                else {
                    if (_fl.isBusy() || _fr.isBusy() || _rl.isBusy() || _rr.isBusy()) {
                        // one of wheels still moving, need wait
                    }
                    else {
                        break;
                    }
                }

                int curPosFrontLeft = _fl.getCurrentPosition();
                int curPosFrontRight = _fr.getCurrentPosition();
                int curPosRearLeft = _rl.getCurrentPosition();
                int curPosRearRight = _rr.getCurrentPosition();

                double distanceRemainingFrontLeft = newFrontLeftTarget - curPosFrontLeft;
                double distanceRemainingFrontRight = newFrontRightTarget - curPosFrontRight;
                double distanceRemainingRearLeft = newRearLeftTarget - curPosRearLeft;
                double distanceRemainingRearRight = newRearRightTarget - curPosRearRight;

                double maxDistanceRemaining = Math.max(Math.max(Math.abs(distanceRemainingFrontLeft), Math.abs(distanceRemainingFrontRight)),
                        Math.max(Math.abs(distanceRemainingRearLeft), Math.abs(distanceRemainingRearRight)));

                if (maxDistanceRemaining <= thresholdDistance) {
                    if (!useIMUKeepStraight) {
                        softStopPower = speed * (maxDistanceRemaining / thresholdDistance);

                        // Apply the soft stop power
                        if (softStopPower < thresholdPower) {
                            softStopPower = thresholdPower;
                        }

                        _fl.setPower(frontLeftPower * softStopPower);
                        _fr.setPower(frontRightPower * softStopPower);
                        _rl.setPower(rearLeftPower * softStopPower);
                        _rr.setPower(rearRightPower * softStopPower);
                    }
                    else {
                        softStopPower = speed * (maxDistanceRemaining / thresholdDistance);
                        // Apply the soft stop power
                        if (softStopPower < thresholdPower) {
                            softStopPower = thresholdPower;
                        }
                        // Get the current yaw angle from the IMU sensor
                        currentYaw = _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        // Calculate the error between the target and current yaw angles
                        error = targetYaw - currentYaw;

                        // the Yaw value is 0->90->180 when turn left, 0 -> -90 -> -180 when turn right
                        // if the value (target - current) is negative, need right turn: make right wheel less power
                        // if (target - current) is positive, need left turn: left wheel will get less power
                        // the bigger (target - current), the bigger change to the value (get smaller power)
                        leftPower = frontLeftPower * softStopPower;
                        rightPower = frontRightPower * softStopPower;
                        leftPower2 = frontLeftPower * softStopPower;
                        rightPower2 = frontRightPower * softStopPower;
                        if (Math.abs(error) >= RobotDataBase.yawExpectedDelta) {
                            if (error > 0) {
                                // left wheel less power
                                leftPower2 = leftPower * (1.0 - Math.abs(error) / baseError);
                            } else if (error < 0) {
                                // right wheel less power
                                rightPower2 = rightPower * (1.0 - Math.abs(error) / baseError);
                            }
                        }
                    }
                }
                else {
                    if (pidErrorAdjust) {
                        // calc the PID error and adjust the power
                        double error1 = distanceRemainingFrontLeft;
                        double error2 = distanceRemainingFrontRight;
                        double error3 = distanceRemainingRearLeft;
                        double error4 = distanceRemainingRearRight;
                        totalError = error1 + error2 + error3 + error4;
                        double powerAdjustment1 = kP * error1 + kI * totalError + kD * (error1 - lastError);
                        double powerAdjustment2 = kP * error2 + kI * totalError + kD * (error2 - lastError);
                        double powerAdjustment3 = kP * error3 + kI * totalError + kD * (error3 - lastError);
                        double powerAdjustment4 = kP * error4 + kI * totalError + kD * (error4 - lastError);
                        lastError = totalError;
                        _fl.setPower(frontLeftPower - powerAdjustment1);
                        _fr.setPower(frontRightPower - powerAdjustment2);
                        _rl.setPower(rearLeftPower - powerAdjustment3);
                        _rr.setPower(rearRightPower - powerAdjustment4);
                    }
                    else if (useIMUKeepStraight) {
                        /*
                        if (false) { // this method not work for now, please see the true part
                            // Calculate the integral of the error
                            integral += error * loopTime;
                            // Calculate the derivative of the error
                            derivative = (error - lastError) / loopTime;
                            // Calculate the correction value based on the error, the integral, the derivative, and the PID gains
                            correction = error * kP + integral * kI + derivative * kD;
                            // Adjust the power of the left and right wheels using the correction value
                            if (direction.equals("wheel_forward")) {
                                leftPower = frontLeftPower * speed + correction;
                                rightPower = frontRightPower * speed + correction;
                            } else if (direction.equals("wheel_back")) {
                                leftPower = frontLeftPower * speed + correction;
                                rightPower = frontRightPower * speed - correction;
                            }
                            leftPower = Range.clip(leftPower, minPower, maxPower);
                            rightPower = Range.clip(rightPower, minPower, maxPower);
                            leftPower2 = leftPower;
                            leftPower3 = leftPower;
                            rightPower2 = rightPower;
                            rightPower3 = rightPower;
                            // Make sure the power of the wheels is not below the minimum threshold
                            if (Math.abs(leftPower) < thresholdPower) {
                                leftPower2 = Math.signum(leftPower) * thresholdPower;
                            }
                            if (Math.abs(rightPower) < thresholdPower) {
                                rightPower2 = Math.signum(rightPower) * thresholdPower;
                            }
                        }
                         */

                        // Update the previous error with the current error
                        lastError = error;
                        // Get the current yaw angle from the IMU sensor
                        currentYaw = _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        // Calculate the error between the target and current yaw angles
                        error = targetYaw - currentYaw;

                        if (true) {
                            // the Yaw value is 0->90->180 when turn left, 0 -> -90 -> -180 when turn right
                            // if the value (target - current) is negative, need right turn: make right wheel less power
                            // if (target - current) is positive, need left turn: left wheel will get less power
                            // the bigger (target - current), the bigger change to the value (get smaller power)
                            leftPower = frontLeftPower * speed;
                            rightPower = frontRightPower * speed;
                            leftPower2 = frontLeftPower * speed;
                            rightPower2 = frontRightPower * speed;
                            if (Math.abs(error) >= RobotDataBase.yawExpectedDelta) {
                                if (direction.equals("wheel_forward")) {
                                    if (error > 0) {
                                        // left wheel less power
                                        leftPower2 = leftPower * (1.01 - Math.abs(error) / baseError);
                                    } else if (error < 0) {
                                        // right wheel less power
                                        rightPower2 = rightPower * (1.01 - Math.abs(error) / baseError);
                                    }
                                }
                                else if (direction.equals("wheel_back")) {
                                    if (error > 0) {
                                        // right wheel less power
                                        rightPower2 = rightPower * (1.01 - Math.abs(error) / baseError);
                                    } else if (error < 0) {
                                        // left wheel less power
                                        leftPower2 = leftPower * (1.01 - Math.abs(error) / baseError);
                                    }
                                }
                            }
                        }

                        // Set the power of the four wheels
                        _fl.setPower(leftPower2);
                        _fr.setPower(rightPower2);
                        _rl.setPower(leftPower2);
                        _rr.setPower(rightPower2);
                    }
                }


                // Display it for the driver.
                op.telemetry.addData("soft power", " %4f: ", softStopPower);
                op.telemetry.addData("useIMUKeepStraight", useIMUKeepStraight);
                op.telemetry.addData("direction: ", direction);
                op.telemetry.addData("DistanceInches: ", distanceInches);
                op.telemetry.addData("maxDistanceRemaining: ", maxDistanceRemaining);
                op.telemetry.addData("thresholdDistance: ", thresholdDistance);
                op.telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                op.telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d", curPosFrontLeft, curPosFrontRight, curPosRearLeft, curPosRearRight);
                op.telemetry.addData("Power", "left: %7f right: %7f, left2: %7f right2: %7f", leftPower, rightPower, leftPower2, rightPower2);
                op.telemetry.addData("Power", "actual left: %7f right: %7f, correction: %7f", _fl.getPower(), _fr.getPower(), correction);
                op.telemetry.addData("Yaw", "currentYaw: %7f targetYaw: %7f error: %7f", currentYaw, targetYaw, error);
                op.telemetry.update();

                // sleep for a short period to allow the motor to move
                //sleep(1);
            }
        }
        else {
            // if want to use total steps in here, need copy the related above code to here. we are not using this for now
            while ((wheelRunTime.milliseconds() < timeoutMilliSeconds) &&
                    (_fl.isBusy() && _fr.isBusy() && _rl.isBusy() && _rr.isBusy())) {

                // Display it for the driver.
                op.telemetry.addData("mark position: ", markPosition);
                op.telemetry.addData("direction: ", direction);
                op.telemetry.addData("DistanceInches: ", distanceInches);
                op.telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                op.telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d", _fl.getCurrentPosition(), _fr.getCurrentPosition(), _rl.getCurrentPosition(), _rr.getCurrentPosition());
                op.telemetry.update();

                // sleep for a short period to allow the motor to move
                sleep(1);
            }
        }

        // Stop all motion;
        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);

        // Turn off RUN_TO_POSITION
        // RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
        _fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(100);   // optional pause after each move.
    }

    public void wheelBalance(String[] splitStrings) throws InterruptedException {
        boolean conditionMatch = true;
        if (splitStrings.length > 1) {
            if (splitStrings[1].equals("reset")) {
                targetFrontLeft = _fl.getCurrentPosition();
                targetFrontRight = _fr.getCurrentPosition();
                targetRearLeft = _rl.getCurrentPosition();
                targetRearRight = _rr.getCurrentPosition();
                return;
            }
            conditionMatch = false;
            int i = 1;
            while (i < splitStrings.length) {
                if (splitStrings[i].equals(workingMode)) {
                    conditionMatch = true;
                    break;
                }
                i = i + 1;
            }
        }
        if (!conditionMatch)
            return;
        ElapsedTime wheelRunTime = new ElapsedTime();
        double power = 0.2;

        _fl.setTargetPosition(targetFrontLeft);
        _fr.setTargetPosition(targetFrontRight);
        _rl.setTargetPosition(targetRearLeft);
        _rr.setTargetPosition(targetRearRight);

        _fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _fl.setPower(power);
        _fr.setPower(power);
        _rl.setPower(power);
        _rr.setPower(power);

        while ((wheelRunTime.milliseconds() < 1500) && (_fl.isBusy() || _fr.isBusy() || _rl.isBusy() || _rr.isBusy())) {
            sleep(1);
        }
        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);
        _fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    class ThreadWheel implements Runnable {
        private volatile boolean exitThread = false;
        public void run() {
            try {
                while (!exitThread) {
                    controlWheels();
                    sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
            }
        }

        public void stop() {
            exitThread = true;
        }
    }

    class ThreadArm implements Runnable {
        private volatile boolean exitThread = false;
        public void run() {
            try {
                while (!exitThread) {
                    controlArm();
                    sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
            }
        }

        public void stop() {
            exitThread = true;
        }
    }

    class ThreadReplay implements Runnable {
        public String action;
        public void run() {
            try {
                replayAction(action);
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
            }
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() throws InterruptedException {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (useCamera) {
            builder.setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }   // end method initAprilTag()

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    cameraSetManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            op.telemetry.addData("Camera", "Waiting");
            op.telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            op.telemetry.addData("Camera", "Ready");
            op.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!op.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void    cameraSetAutoMode() throws InterruptedException {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        ElapsedTime timeWaitAi = new ElapsedTime();

        // Make sure camera is streaming before we try to set the mode
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            op.telemetry.addData("Camera", "Waiting");
            op.telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && timeWaitAi.milliseconds() < 60000) {
                op.telemetry.addData("Camera waiting", timeWaitAi.milliseconds());
                op.telemetry.update();
                sleep(20);
            }
            op.telemetry.addData("Camera", "Ready");
            op.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!op.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Auto) {
                exposureControl.setMode(ExposureControl.Mode.Auto);
                sleep(50);
            }
        }
    }

    double getDistanceAverageByInch(DistanceSensor sensor, int count) throws InterruptedException {
        int num = count <= 0 || count > 100 ? 100 : count;
        double[] distanceRecords = new double[num];
        double timeBegin = timeSinceStart.milliseconds();
        for (int i = 0; i < num; i++) {
            double curDistance = sensor.getDistance(DistanceUnit.INCH);
            if (curDistance > 100) {
                // distance sensor may get very big value, ignore it, try get the value one more time
                curDistance = sensor.getDistance(DistanceUnit.INCH);
            }
            op.telemetry.addData("Distance ", curDistance);
            op.telemetry.addData("Time ", (int)(timeSinceStart.milliseconds() - timeBegin));
            op.telemetry.update();
            distanceRecords[i] = curDistance;
            //sleep(1);
        }

        double total = 0;
        boolean ignoreSmallestAndBiggest = false;

        if (ignoreSmallestAndBiggest && num >= 3) {
            Arrays.sort(distanceRecords);
            // ignore the smallest and biggest value
            for (int i = 1; i < num - 1; i++) {
                total += distanceRecords[i];
            }
            return total / (num - 2);
        }
        else {
            for (int i = 0; i < num; i++) {
                total += distanceRecords[i];
            }
            return total / num;
        }
    }

    boolean aiGetMarkPosition(String[] splitStrings) throws InterruptedException {
        if (robotData.useDistanceSensorToDetectMark) {
            return aiGetMarkPositionByDistanceSensor(splitStrings);
        }
        else {
            return aiGetMarkPositionByTeamprop(splitStrings);
        }
    }

    boolean aiGetMarkPositionByDistanceSensor(String[] splitStrings) throws InterruptedException {
        double curDistance = 0;
        curDistance = getDistanceAverageByInch(_distanceGrip, robotData.getDistanceMultipleTimes ? robotData.numGetDistance : 1);

        // only for debug
        if (!autoMode && robotData.debugMode) {
            while (!op.gamepad2.right_bumper) {
                curDistance = _distanceGrip.getDistance(DistanceUnit.INCH);
                op.telemetry.addData("Press pad2 right_bumper to exit this loop.", "");
                op.telemetry.addData("Distance ", curDistance);
                op.telemetry.update();
                sleep(1);
            }
            return false;
        }

        if (!robotData.checkMarkPositionClose) {
            if (curDistance > 100) {
                // distance sensor may get very big value, ignore it, try get the value one more time
                curDistance = getDistanceAverageByInch(_distanceGrip, robotData.getDistanceMultipleTimes ? robotData.numGetDistance : 1);
            }
            // distance sensor is not accurate when detecing the object far than 20 inches ???
            // from what observed in the data, the returned value is from 18~20, 322 when there is nothing in the center position
            // should be 28, return as 23
            if (curDistance > 22 && curDistance < 30) {
                // found it!
                markPosition = 1; // center;
                markSeen = true;
                op.telemetry.addData("Distance ", curDistance);
                op.telemetry.addData("Mark type ", markPosition);
                op.telemetry.update();
                return true;
            }
        }

        recordXY = true;
        if (robotData.checkMarkPositionClose) {
            // check the center position close to the team prop, sometimes the distance sensor may detect not correctly
            replayActionList(robotData.presetActionsDetectMoveForwardOnly);

            curDistance = getDistanceAverageByInch(_distanceGrip, robotData.getDistanceMultipleTimes ? robotData.numGetDistance : 1);
            if (curDistance > 13 && curDistance < 18) { //usually is about 15 inch
                // found it;
                markPosition = 1; //center
                markSeen = true;
                op.telemetry.addData("Distance ", curDistance);
                op.telemetry.addData("Mark type ", markPosition);
                op.telemetry.update();
                recordXY = false;
                return true;
            }
        }
        if (workingMode.contains("redfar") || workingMode.contains("bluenear"))
        {
            // go to left side to take a look!
            if (robotData.checkMarkPositionClose) {
                replayActionList(robotData.presetActionsDetectMoveLeftOnly);
            }
            else {
                replayActionList(robotData.presetActionsDetectMoveLeft);
            }

            curDistance = getDistanceAverageByInch(_distanceGrip, robotData.getDistanceMultipleTimes ? robotData.numGetDistance : 1);
            op.telemetry.addData("After move to left ", curDistance);
            if (curDistance > 6 && curDistance < 15) {
                // found it;
                markPosition = 0; //left
                markSeen = true;
            }
            else {
                // not found;
                markPosition = 2; // right;
            }
            //replayActionList(robotData.presetActionsDetectMoveBackFromRight);
        }
        else {
            // go to right side to take a look!
            if (robotData.checkMarkPositionClose) {
                replayActionList(robotData.presetActionsDetectMoveRightOnly);
            }
            else {
                replayActionList(robotData.presetActionsDetectMoveRight);
            }
            curDistance = getDistanceAverageByInch(_distanceGrip, robotData.getDistanceMultipleTimes ? robotData.numGetDistance : 1);
            op.telemetry.addData("After move to right ", curDistance);
            if (curDistance > 6 && curDistance < 15) {
                // found it;
                markPosition = 2; //right
                markSeen = true;
            }
            else {
                // not found;
                markPosition = 0; // left;
            }
            op.telemetry.addData("Mark type ", markPosition);
            op.telemetry.update();
            //replayActionList(robotData.presetActionsDetectMoveBackFromLeft);
        }
        recordXY = false;
        return true;
    }

    boolean aiGetMarkPositionByApriltag(String[] splitStrings) throws InterruptedException {
        if (!useCamera) {
            return false;
        }
        double distanceToCenter = 0;
        if (splitStrings.length > 1) {
            distanceToCenter = Double.parseDouble(splitStrings[1]);
        }
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        ElapsedTime timeWaitAi = new ElapsedTime();

        int type = -1;
        while (!targetFound && timeWaitAi.milliseconds() < 500) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            op.telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    op.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    op.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    op.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    op.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                } else {
                    op.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    op.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            if (targetFound) {
                // get the mark position through camparing the x value
                if ((desiredTag.ftcPose.x + distanceToCenter) <= -6) {
                    type = 0; //left
                }
                else if ((desiredTag.ftcPose.x + distanceToCenter) >= 6) {
                    type = 2; // right
                }
                else {
                    type = 1; // center
                }
                markSeen = true;
                op.telemetry.addData("Mark type ", type);
            }
            op.telemetry.update();
            sleep(20);
        }

        if (!targetFound && robotData.doubleCheckMark) {
            timeWaitAi.reset();
            int i = 0;
            while (!targetFound && timeWaitAi.milliseconds() < 1000) {
                i = i + 1;
                // try to change some parameters of the camera, to see is there a improved chance
                cameraSetManualExposure(10*i, 5*i);
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                op.telemetry.addData("# AprilTags Detected", currentDetections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        op.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        op.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        op.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        op.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        }
                    } else {
                        op.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        op.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }   // end for() loop

                if (targetFound) {
                    // get the mark position through camparing the x value
                    if ((desiredTag.ftcPose.x + distanceToCenter) <= -6) {
                        type = 0; //left
                    } else if ((desiredTag.ftcPose.x + distanceToCenter) >= 6) {
                        type = 2; // right
                    } else {
                        type = 1; // center
                    }
                    markSeen = true;
                    op.telemetry.addData("Mark type ", type);
                }
                op.telemetry.update();
            }
            sleep(1);
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

        if (type == -1)
            return false;
        else {
            markPosition = type;
            return true;
        }
    }

    public boolean aiDoubleCheckMarkPosition(String[] splitStrings) throws InterruptedException {
        if (!useCamera) {
            return false;
        }
        if (markSeen || markPosition != 0) // had seen the mark, no need to take another check
            return true;

        if (workingMode.contains("rednear")) {
            replayActionList(robotData.markDoubleCheckPosition[0]);
        }
        else if (workingMode.contains("redfar")) {
            replayActionList(robotData.markDoubleCheckPosition[1]);
        }
        else if (workingMode.contains("bluenear")) {
            replayActionList(robotData.markDoubleCheckPosition[2]);
        }
        else if (workingMode.contains("bluefar")) {
            replayActionList(robotData.markDoubleCheckPosition[3]);
        }
        return true;
    }

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public void aiRunToTag(String[] splitStrings) throws InterruptedException {
        int tagId = Integer.parseInt(splitStrings[1]);
        double distance = Double.parseDouble(splitStrings[2]);
        aiRunToTag(tagId, distance);
    }

    public void aiRunToTag(int tagId, double distanceToTarget) throws InterruptedException {
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        ElapsedTime timeWaitAi = new ElapsedTime();

        while (!targetFound && timeWaitAi.milliseconds() < 1500) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            op.telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    op.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    op.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    op.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    op.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    if ((DESIRED_TAG_ID < 0) || (detection.id == tagId)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                } else {
                    op.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    op.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            op.telemetry.update();
        }

        if (targetFound) {
            double forwardDistance = desiredTag.ftcPose.y - distanceToTarget;
            String sDirection = "wheel_forward";
            if (forwardDistance < 0)
                sDirection = "wheel_back";
            String sCommand = String.format("%s @%6.7f @0.3", sDirection, Math.abs(forwardDistance));
            //telemetry.addLine(String.format("\n play action: %s", sCommand));
            //telemetry.update();
            //waitElapsedTime(5000);
            replayAction(sCommand);

            sDirection = "wheel_right";
            if (desiredTag.ftcPose.x < 0)
                sDirection = "wheel_left";
            sCommand = String.format("%s @%6.7f @0.3", sDirection, Math.abs(desiredTag.ftcPose.x));
            //telemetry.addLine(String.format("\n play action: %s", sCommand));
            //telemetry.update();
            //waitElapsedTime(5000);
            replayAction(sCommand);
        }

    }

    public void aiRunToTag1(int tagId, double distanceToTarget) throws InterruptedException {
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((tagId < 0) || (detection.id == tagId))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                op.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            op.telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            op.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            op.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            op.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            op.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            op.telemetry.addData(">","Drive using joysticks to find valid target\n");
        }

        // If we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - distanceToTarget);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            op.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            op.telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
        }

        sleep(10);

    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        _fl.setPower(leftFrontPower);
        _fr.setPower(rightFrontPower);
        _rl.setPower(leftBackPower);
        _rr.setPower(rightBackPower);
    }


    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private String TFOD_MODEL_FILE_NUMBER  = "/sdcard/FIRST/tflitemodels/shape.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/box2024school.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "1",
            "2",
    };
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() throws InterruptedException {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (useCamera) {
            builder.setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        cameraSetAutoMode();
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        op.telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            op.telemetry.addData(""," ");
            op.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            op.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            op.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    boolean aiGetMarkPositionByTeamprop(String[] splitStrings) throws InterruptedException {
        if (!useCamera) {
            return false;
        }
        if (markSeen) {
            op.telemetry.addData("Already detected", markPosition);
            op.telemetry.update();
            return true;
        }

        double distanceToCenter = 0;
        if (splitStrings.length > 1) {
            distanceToCenter = Double.parseDouble(splitStrings[1]);
        }
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        ElapsedTime timeWaitAi = new ElapsedTime();

        int type = -1;
        xMark = 0;
        yMark = 0;
        markPosition = 0; // default is left, where the camera couldn't take a full picture or no picture at all
        double x = 0 ;
        double y = 0 ;
        int count = 1;
        int i=0;
        while (!targetFound && timeWaitAi.milliseconds() < (debugMode ? 1000 : 500)) {
            i += 1;
            if (i > 5) {
                // not detected the team prop, change to manual mode to get some luck
                cameraSetManualExposure(10*i, 5*i);
            }
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            op.telemetry.addData("# Count", count);
            count += 1;
            op.telemetry.addData("# Objects Detected", currentRecognitions.size());
            op.telemetry.update();
            if (currentRecognitions != null) {
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    xMark = x;
                    yMark = y;

                    op.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    op.telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    op.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    op.telemetry.update();
                    sleep(100);

                    // may detect other object like part of the environment, need compare the width and height
                    if (recognition.getConfidence() > 0.6 && recognition.getWidth() >= 50 && recognition.getWidth() <= 200 &&
                            recognition.getHeight() >= 50 && recognition.getHeight() <= 200) {
                        targetFound = true;
                        xMark = x;
                        yMark = y;
                        markSeen = true;
                        break;
                    }
                }
                op.telemetry.update();
                if (targetFound) {
                    double pos = x+distanceToCenter;
                    if (workingMode.contains("rednear") || workingMode.contains("redfar")) {
                        // get the mark position through camparing the x value
                        if (pos >= 100 && pos <= 300) {
                            type = 1; //center
                        } else if (pos >= 400) {
                            type = 2; // right
                        } else {
                            type = 0; // left
                        }
                    }
                    else {
                        if (pos >= 100 && pos <= 300) {
                            type = 1; //center
                        } else if (pos >= 400) {
                            type = 2; // right
                        } else {
                            type = 0; // left
                        }
                    }
                    op.telemetry.addData("Mark type ", type);
                }
                op.telemetry.update();
                sleep(1);
            }
        }
        // set the camera back to auto mode
        cameraSetAutoMode();

        /*
        if (!targetFound && robotData.doubleCheckMark) {
            timeWaitAi.reset();
            int i = 0;
            while (!targetFound && timeWaitAi.milliseconds() < 500) {
                i = i + 1;
                // try to change some parameters of the camera, to see is there a improved chance
                setManualExposure(10*i, 5*i);
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                op.telemetry.addData("# Objects Detected", currentRecognitions.size());
                if (currentRecognitions != null) {
                    op.telemetry.addData("# Objects Detected", currentRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : currentRecognitions) {
                        x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                        op.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        op.telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        op.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                        if (recognition.getConfidence() > 0.6) {
                            targetFound = true;
                            break;
                        }
                    }
                    op.telemetry.update();
                    if (targetFound) {
                        double pos = x+distanceToCenter;
                        if (workingMode.contains("rednear") || workingMode.contains("redfar")) {
                            // get the mark position through camparing the x value
                            if (pos >= 150 && pos <= 300) {
                                type = 1; //center
                            } else if (pos >= 400 && pos <= 500) {
                                type = 2; // right
                            } else {
                                type = 0; // left
                            }
                        }
                        else {
                            if (pos >= 150 && pos <= 300) {
                                type = 1; //center
                            } else if (pos >= 400 && pos <= 500) {
                                type = 2; // right
                            } else {
                                type = 0; // left
                            }
                        }
                        markSeen = true;
                        op.telemetry.addData("Mark type ", type);
                    }
                    op.telemetry.update();
                }
                sleep(1);
            }
        }
        */

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

        if (type == -1) {
            return false;
        }
        else {
            markPosition = type;
            return true;
        }
    }

    void driveMarkPosition() {
        //String operation = "wheel_forward";
        //String sDistance = markDriveData[markPosition][0];
        //wheel(operation, sDistance, "0.5", 10000);
    }

    void cameraSetting(String[] splitStrings) throws InterruptedException {
        if (splitStrings.length <= 1)
            return;
        if (splitStrings[1].equals("close")) {
            if (useCamera && cameraStreaming) {
                visionPortal.setProcessorEnabled(tfod, false);
                sleep(100);
                visionPortal.close();
                cameraStreaming = false;
            }
        }
    }

    void waitElapsedTime(int waitTime) throws InterruptedException {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < waitTime) {
            //telemetry.addData("Waiting millisecond: ", moreTimeToStart.milliseconds()  );
            //telemetry.update();
            sleep(1);
            continue;
        }
    }

    // turn robot with supporting of IMU
    boolean imuTurn(String[] splitStrings) {
        // Define the desired yaw angle (in degrees)
        // imuturn @90 @0.2
        boolean result = false;
        if (splitStrings.length < 3 || !robotData.useIMU)
            return result;
        double targetYaw = Double.parseDouble(splitStrings[1]);
        double power = Math.abs(Double.parseDouble(splitStrings[2]));

        // Define the proportional gain constant
        double kP = 0.1;

        // Define the minimum and maximum power for the wheels
        double minPower = -power;
        double maxPower = power;

        imuWheelOperationNow = true;

        // Define the tolerance for the error (in degrees)
        double tolerance = 0.25;
        double threasholdPower = 0.1;
        double threasholdDegree = 3;

        ElapsedTime t = new ElapsedTime();
        double error;

        do {
            // Get the current yaw angle from the IMU sensor
            double currentYaw = _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Calculate the error between the target and current yaw angles
            error = targetYaw - currentYaw;

            // Check if the error is within the tolerance
            if (Math.abs(error) < tolerance) {
                // Return true to indicate that the robot is aligned
                result = true;
                break;
            }
            // add soft stop power when closing to the target position
            if (Math.abs(error) < threasholdDegree) {
                minPower = 0-threasholdPower;
                maxPower = threasholdPower;
            }
            // The robot is not aligned with the target yaw angle
            // Calculate the correction value based on the error and the proportional gain
            double correction = error * kP;

            // Adjust the power of the four wheels using the correction value
            double frontLeftPower = Range.clip(correction, minPower, maxPower);
            double frontRightPower = Range.clip(-correction, minPower, maxPower);
            double rearLeftPower = Range.clip(correction, minPower, maxPower);
            double rearRightPower = Range.clip(-correction, minPower, maxPower);

            // Set the power of the four wheels
            _fl.setPower(frontLeftPower);
            _fr.setPower(frontRightPower);
            _rl.setPower(rearLeftPower);
            _rr.setPower(rearRightPower);
            logAction("imuTurn");
        } while (Math.abs(error) >= tolerance && t.milliseconds() <= 10000);

        // The robot is aligned with the target yaw angle
        // Set the power of the four wheels to zero
        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);
        imuWheelOperationNow = false;

        return result;
    }
}



