package org.firstinspires.ftc.teamcode;

public class RobotDataBase {
    public static String defaultType = "arm";
    public String type = defaultType;
    public String workingMode = "rednear";
    public boolean parkingCenter = true;
    public boolean getOneWhitePixels = true;
    public boolean getWhitePixels = false;
    public boolean parkingAtBackDrop = true;
    public double fastWheelSpeedManual = 1.00;
    public boolean checkMarkPositionClose = false;
    public boolean replayInNewThreadWhenUsingThreadKeyword = true;
    public boolean getDistanceMultipleTimes = true;
    public boolean sliderUseSoftStop = true;
    public boolean debugMode = false;
    public boolean useNewManualDriveMethod = false;
    public boolean useFastLifter = true;
    public boolean testWheelBalance = false;
    public boolean sliderUpAfterDrop = true;
    public boolean useDistanceSensorToDetectMark = false;
    public boolean hangFarSide = false;
    public boolean continueRunToBoard = true;
    public boolean useIMU = true;
    public boolean useTwoWayDoor = true;
    public boolean farSideUsingCenterLane = true;

    public static int sleepMainThreadMilliSeconds = 10;
    public static double yawErrorRatioBase = 90;
    public static double yawErrorBigValue = 30;
    public static double yawExpectedDelta = 0.25; // 0 ~ 180
    public int numGetDistance = 1;
    public boolean scriptStopable = false;

    public String[][][] markRedNear;

    public String[] markRedLeftIndex1 = {};

    public String[][][] markRedFar = { {{}} };

    public String[] markRedFarRightIndex1 = {};

    public String[] markRedFarRightIndex5 = {};

    public String[][][] markBlueNear = { {{}} };

    public String[] markBlueRightIndex1 = {};

    public String[][][] markBlueFar = { { {}} };

    // rednear = 0
    // readfar = 1
    // bluenear = 2
    // bluefar = 3
    public String[][] markDoubleCheckPosition = {
            {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // rednear
            {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // redfar
            {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // bluenear
            {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // bluefar
    };

    public boolean useDistanceSensor = true;
    public boolean useLifter = true;
    public boolean dropWhitePixelLater = true;
    public boolean useGrip2 = false;
    public int minTimeOfTwoOperations = 150; //milliseconds, 0.15 second
    public boolean useFastMethod = true;
    public boolean debugReplayActionListWithSleep = false;
    public boolean doubleCheckMark = true;
    public boolean useNewDroplowPosition = true;
    public boolean useBox1 = false;
    public boolean useArm1 = false;
    public boolean sliderRow0PositionLower = true;

    public double sliderDefaultPower = 1.0;
    public int sliderPerInch = 100;
    // high - firstrow = 1500 - 875 = 625 = about 17.25 inch - 11inch = 6.25inch  // per inch = 100
    public int sliderMinPosition = 0;
    public int sliderMaxPosition = 2200;
    public int sliderInitialPosition = 0; // may change every time
    public int sliderFirstRowPosition = 875; // first pixel drop position when no pixel on the board
    public int sliderHighPosition = 1500;
    public int sliderStepSize = 5 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
    public int sliderSmallStepSize = 4 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
    public int sliderDownStepSize = 4 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
    public int sliderRow0Position = sliderFirstRowPosition;
    public int sliderRow1FarPosition = sliderFirstRowPosition;
    public int sliderRow1Position = sliderFirstRowPosition;
    public int sliderRow2Position = sliderFirstRowPosition + 3 * sliderPerInch;
    public int sliderRow3Position = sliderFirstRowPosition + 6 * sliderPerInch;
    public int sliderRow4Position = sliderFirstRowPosition + 9 * sliderPerInch;
    public int sliderRow5Position = sliderFirstRowPosition + 12 * sliderPerInch;
    public int sliderLine1Position = sliderRow4Position;
    public int sliderLine2Position = sliderFirstRowPosition + 18 * sliderPerInch;

    public int lifterMaxPosition = 1200;

    // grip data
    public double perStepSizeGrip = 0.01;
    public double gripMinPosition = 0.00;
    public double gripMaxPosition = 1.00;
    public double gripOpenSmallPosition = 0.49; //0.52; // old value is 0.53;
    public double gripOpenMiddlePosition = 0.44;
    public double gripOpenLargePosition = 0.34;
    public double gripOpenVeryLargePosition = 0.34;
    public double gripClosePosition = 0.56; //0.53; //0.59;
    public double gripDefaultPosition = gripClosePosition; // close, old value is 0.60

    public double grip2MinPosition = 0.00;
    public double grip2MaxPosition = 1.00;
    public double grip2OpenSmallPosition = 0.36;
    public double grip2OpenLargePosition = 0.59;
    public double grip2ClosePosition = 0.31;
    public double grip2DefaultPosition = gripClosePosition; // close, old value is 0.60

    public double armMinPosition = 0.00;
    public double armMaxPosition = 1.00;
    public double armUpPosition = 0.86; //0.63; //0.7350; // near the slider and ready to drop the pixel
    public double armVerticalPosition = 0.55;
    public double armDownPosition = 0.19; // 0.20; // ground
    public double armHigh5Position = 0.22; //0.23; //0.16945;
    public double armHigh4Position = 0.215; //armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 3;
    public double armHigh3Position = armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 2;
    public double armHigh2Position = armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 1;
    public double armHigh1Position = armDownPosition;
    public double armMoveReadyPosition = 0.78;
    public double armFirstRowPosition = 0.70;
    public double armSecondRowPosition = 0.70;
    public double armDefaultPosition = armMoveReadyPosition; //0.66; // close to the slider but not so close

    public double boxMinPosition = 0.00;
    public double boxMaxPosition = 1.00;
    public double boxPickupPosition = 0.58; // 0.60; //0.39; //0.42; //0.49;
    public double boxPixelSlidePosition = 0.61; //0.63; //0.55; // pixel may not slide to the bottom of the box, need a pixel slide position
    public double boxDropPosition = 0.83; //0.85; //0.86; //0.70; //0.65; //0.72; //0.75 is about vertical to the ground
    public double boxDropLowPosition= 0.84; //0.86; //0.87; //0.69; //0.73; // lifter is just one step high
    public double boxDropLowLeftPosition= 0.84; //0.86; //0.87; //0.69; //0.73; // lifter is just one step high
    public double boxDefaultPosition = boxPickupPosition;

    public double doorMinPosition = 0.00;
    public double doorMaxPosition = 1.00;
    public double doorOpenPosition = 0.30;
    public double doorOpenOnePosition = 0.30;
    public double doorClosePosition = 0.50;
    public double doorDefaultPosition = doorClosePosition; // close

    public double swiperlifterMinPosition = 0.00;
    public double swiperlifterMaxPosition = 1.00;
    public double swiperlifterUpPosition = 0.20;
    public double swiperlifterDownPosition = 0.80; // ground
    public double swiperlifterHigh5Position = 0.7;
    public double swiperlifterHigh4Position = 0.6;
    public double swiperlifterHigh3Position = 0.5;
    public double swiperlifterHigh2Position = 0.4;
    public double swiperlifterHigh1Position = 0.1355;
    public double swiperlifterDefaultPosition = swiperlifterUpPosition;

    public double lifterarmMinPosition = 0.00;
    public double lifterarmMaxPosition = 1.00;
    public double lifterarmUpPosition = 0.71; // near the slider and ready to drop the pixel
    public double lifterarmDownPosition = 0.1195; // ground
    public double lifterarmDefaultPosition = 0.5;

    public double droneMinPosition = 0.00;
    public double droneMaxPosition = 1.00;
    public double droneOpenPosition = 0.34;
    public double droneClosePosition = 0.56;
    public double droneDefaultPosition = droneClosePosition;

    public double distanceTurn90Degree = 24.35; //15.655;


    // "wheel_forward @10 @0.5", wheel_back 10inch and speed is 0.5
    // wheel_left/wheel_right/wheel_back
    // platform and shoulder elbow remain still, position / direction not changed

    // "wheel_turn_left @14 @0.3" turn left about 90 degree at speed 0.3
    // "wheel_turn_left @28 @0.2" turn about 180 degree at speed 0.2
    // platform / shoulder / elbow direction will change at same time

    // "platform_left @10" : platform turn left 10 times the perStepSize
    // "platform_right @20
    // "shoulder_up @10" "shoulder_down @20"
    // shoulder up or down 10 times the perStepSize
    // "elbow_up @10" "elbow_down @20"
    //
    // "both_min" shoulder and elbow will go down to the lowest position, ready to grab cone
    // "both_max" shoulder and elbow will go up to the highest position, ready to put cone to the pole

    // "position_shoulder @0.4", shoulder setPosition directly to 0.4
    // "position_platform @0.8", platform setPosition directly to 0.8
    // "position_elbow @ 0.5", elbow setPosition directly to 0.5

    // "grip_max" "grip_min" "grip_open @10" "grip_close @10"
    //

    // "slider @0.2" slider operating at power 0.2
    // "slider @up @0.5" slider up to step size with power 0.5
    // "slider @max @0.5" slider up to max with power 0.5

    // "ai_park" get park destination using AI
    // "zoom @1.5" zoom_in ratio is 1.5
    public String[] presetActionsAutoModeInitBeforeStart = {
            //"box @pickup"
    };
    public String[] presetActionsManualModeInitBeforeStart = {
            //"box @pickup"
    };

    public String[] presetActionsInitAfterStartNonAutoMode = {
            //"arm @moveready",
            //"drone @close"
    };

    public String[] presetActionsLeft = {
            "sleep @1"
    };

    public String[] presetActionsDropPurpleOnly = {
            "grip @close",
            "door @close",
            "ai_getmarkposition @5",
            "sleep @200",
            "box @pickup",
            "arm @vertical",
            "nextstep @mark0",
            "nextstep @mark1",
            "arm @down",
            "sleep @600",
            "grip @small",
            "sleep @200",
            "arm @moveready",
            "sleep @300",
            "grip @close",
    };

    public String[] presetActionsRedNear = {
            "grip @close",
            "door @close",
            "ai_getmarkposition @5",
            "sleep @200",
            "box @pickup",
            "arm @vertical",
            "nextstep @mark0",
            "nextstep @mark1",
            "arm @down",
            "sleep @600",
            "grip @small",
            "sleep @200",
            "arm @moveready",
            "sleep @300",
            "grip @close",
            "nextstep @mark2",
            "nextstep @mark5",
            "sleep @100",
            "nextstep @mark3",
            "nextstep @mark4",
            "balance @redfar @bluefar",
            "distanceslider @0.1 @0.2 @2000",
            "arm @vertical",
            workingMode.contains("near") ? "slider @firstrow": "slider @high2",
            "sleep @300",
            "box @droplow",
            "sleep @1000",
            "door @open",
            "sleep @500",
            "wheel_forward @3 @0.2",
            "box @pickup",
            "slider @min",
            "door @close",
            "arm @moveready",
            "nextstep @mark6",
            //"wheel_back @12 @0.3"
            //"nextstep @presetActionsGetPixels",
    };

    public String[] presetActionsRedFar = presetActionsRedNear;

    public String[] presetActionsBlueNear = presetActionsRedNear;

    public String[] presetActionsBlueFar = presetActionsRedNear;

    public String[] presetActionsGetOnePixelFromStack = {};
    public String[] presetActionsGetTwoPixelFromStack = {};

    public String[] presetActionsGetWhitePixels = {};

    public String[] presetActionsGetAndDropPixels = {
            "grip @close",
            "sleep @500",
            "arm @up",
            "sleep @1000",
            "grip @small",
            //"sleep @100",
            //"arm @default",
            //"sleep @300",
            //"box @pixelslide",
            //"sleep @200",
            //"box @pickup"
    };

    public String[] presetActionsStep2_left = {
            "wheel_right @31 @0.2",
            //"elbow_up @2 @0.2"
    };

    public String[] presetActionsStep2_right = {
            "wheel_left @30.5 @0.25",
            //"elbow_up @2 @0.2"
    };

    public String[] presetActionsAutoE2E = {
    };

    public String[] presetActionsPad1Up = {
            "distanceslider @0.1 @0.3 @3000"
    };

    public String[] presetActionsPad1Down = {
            "distanceslider @0.1 @0.3 @3000"
    };

    public String[] presetActionsPad1Left = {
            //"distancegrip @2 @0.3"
    };

    public String[] presetActionsPad1Right = {
            //"distancegrip @3 @0.3"
    };

    public String[] presetActionsPad1X = {
            "wheel_left @6 @0.3"
    };
    public String[] presetActionsPad1Y = {
            "arm @vertical"
    };

    public String[] presetActionsPad1A = {
            "arm @moveready"
    };

    public String[] presetActionsPad1B = {
            "wheel_right @6 @0.3"
    };

    public String[] presetActionsPad1UpWithRightBumper = {
            "nextstep @mark0",
            "nextstep @mark1",
            "recordposition @stop",
    };
    public String[] presetActionsPad1DownWithRightBumper = {
            "nextstep @presetActionsRedNear"
    };
    public String[] presetActionsPad1LeftWithRightBumper = {
            "grip @close",
            "door @close",
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            "ai_getmarkposition @5",
    };
    public String[] presetActionsPad1RightWithRightBumper = {
            "arm @down",
            "sleep @600",
            "grip @small",
            "sleep @200",
            "arm @moveready",
            "sleep @300",
            "grip @close",
    };
    public String[] presetActionsPad1UpWithLeftBumper = {
            "nextstep @mark5",
            "sleep @100",
            "nextstep @mark3",
            "nextstep @mark4",
            "balance @redfar @bluefar",
            "distanceslider @0.1 @0.2 @1500",
            "arm @vertical",
            "slider @high2",
            "sleep @300",
            "box @droplow",
    };
    public String[] presetActionsPad1DownWithLeftBumper = {
            "wheel_forward @3 @0.2",
            "box @pickup",
            "slider @min",
            "arm @moveready",
            "nextstep @mark6",
    };
    public String[] presetActionsPad1LeftWithLeftBumper = {
            "nextstep @mark2",
    };
    public String[] presetActionsPad1RightWithLeftBumper = {
            "door @open",
            "sleep @1000",
            "door @close",
    };
    public String[] presetActionsPad1XWithLeftBumper = {
            //"motor @frontleft @0.3"
            "ai_getmarkposition"
            //"drone @0.01"
    };
    public String[] presetActionsPad1YWithLeftBumper = {
            //"motor @frontright @0.3"
            //"ai_doublecheckmarkposition"
            "drone @-0.01"
    };
    public String[] presetActionsPad1AWithLeftBumper = {
            "drone @close"
    };
    public String[] presetActionsPad1BWithLeftBumper = {
            "drone @open"
    };
    public String[] presetActionsPad1XWithRightBumper = {
            "wheel_turn_left @90 @0.3"
    };
    public String[] presetActionsPad1YWithRightBumper = {
            "wheel_turn_left @180 @0.3"
    };
    public String[] presetActionsPad1AWithRightBumper = {
            "wheel_turn_right @90 @0.4"
    };
    public String[] presetActionsPad1BWithRightBumper = {
            "wheel_turn_right @180 @0.4"
    };

    public String[] presetActionsWheelTurnLeft = {
            "wheel_turn_left @90 @0.2"
    };

    public String[] presetActionsWheelTurnRight = {
            "wheel_turn_right @90 @0.2"
    };

    public String[] presetActionsPad1Back = {
            "drone @open"
    };

    public String[] presetActionsPad1Start = {
            "log"
    };
    public String[] presetActionsPad1LeftTrigger = {
    };
    public String[] presetActionsPad1RightTrigger = {
    };
    public String[] presetActionsPad1LeftBumper = {
    };
    public String[] presetActionsPad1RightBumper = {
    };

    public String[] presetActionsPad2Up = {
            "arm @vertical",
            "slider @up",
            "box @drop"
    };
    public String[] presetActionsPad2Down = {
            "slider @down",
            "box @pickup"
    };
    public String[] presetActionsPad2Left = {
            "box @pickup"
    };
    public String[] presetActionsPad2Right = {
            "box @drop"
    };

    public String[] presetActionsPad2UpWithLeftBumper = {
            "slider @0.1"
    };
    public String[] presetActionsPad2DownWithLeftBumper = {
            "slider @-0.1"
    };
    public String[] presetActionsPad2LeftWithLeftBumper = {
            "box @0.01"
    };
    public String[] presetActionsPad2RightWithLeftBumper = {
            "box @-0.01"
    };

    public String[] presetActionsPad2UpWithRightBumper = {
            "arm @vertical",
            "slider @up",
            "box @drop",
            "slider @max"
    };
    public String[] presetActionsPad2DownWithRightBumper = {
            "slider @down",
            "box @pickup",
            "slider @min",
            "arm @moveready"
    };
    public String[] presetActionsPad2LeftWithRightBumper = {
            "slider @high2"
    };
    public String[] presetActionsPad2RightWithRightBumper = {
            "slider @high3"
    };

    public String[] presetActionsPad2X = {
            "grip @large"
    };
    public String[] presetActionsPad2Y = {
            "grip @close",
            "sleep @500",
            "arm @up",
            "sleep @1000",
            "grip @small",
            //"sleep @100",
            //"arm @default",
            //"sleep @300",
            //"box @pixelslide",
            //"sleep @200",
            //"box @pickup"
    };
    public String[] presetActionsPad2A = {
            "grip @close",
            "sleep @100",
            "arm @down",
            "sleep @300",
            "grip @large"
    };
    public String[] presetActionsPad2B = {
            "grip @close"
    };

    public String[] presetActionsPad2XWithRightBumper = {
            "door @0.01"
    };

    public String[] presetActionsPad2YWithRightBumper = {
            "grip @small"
    };
    public String[] presetActionsPad2AWithRightBumper = {
            "door @close"
    };
    public String[] presetActionsPad2BWithRightBumper = {
            "door @-0.01"
    };
    public String[] presetActionsPad2XWithLeftBumper = {
            "grip @0.01"
    };
    public String[] presetActionsPad2YWithLeftBumper = {
            "arm @0.01"
    };
    public String[] presetActionsPad2AWithLeftBumper = {
            "arm @-0.01"
    };
    public String[] presetActionsPad2BWithLeftBumper = {
            "grip @-0.01"
    };
    public String[] presetActionsPad2XWithLeftStickX = {
            "arm @high3"
    };
    public String[] presetActionsPad2YWithLeftStickX = {
            "arm @moveready",
    };
    public String[] presetActionsPad2AWithLeftStickX = {
            "arm @vertical",
    };
    public String[] presetActionsPad2BWithLeftStickX = {
            "arm @high4"
    };
    public String[] presetActionsPad2LeftStick = {
            "arm @moveready",
    };
    public String[] presetActionsPad2RightStick = {
            "arm @vertical",
    };
    public String[] presetActionsPad2LeftTrigger = {
            "door @open",
    };
    public String[] presetActionsPad2RightTrigger = {
            "door @open",
            "sleep @800",
            "door @close",
    };
    public String[] presetActionsPad2Back = {
            "log"
    };
    public String[] presetActionsPad2Start = {
            "arm @vertical"
    };
    public String[] presetActionsDefault = {
            "sleep @100"
    };
    public String[] presetActionsPad2LeftstickRightStick = {
            "drone @1"
    };

    public String[] presetActionsDetectMoveRightOnly = {
            "wheel_right @11 @0.3",
            "sleep @100"
    };

    public String[] presetActionsDetectMoveRight = {
            "wheel_forward @12 @0.3",
            "wheel_right @11 @0.3",
            "sleep @100"
    };
    public String[] presetActionsDetectMoveBackFromRight = {
        "wheel_left @11 @0.3 @nowait",
        "wheel_back @12 @0.3 @nowait",
    };

    public String[] presetActionsDetectMoveForwardOnly = {
            //"wheel_forward @12 @0.3 @nowait",
            //"sleep @500",
            "wheel_forward @12 @0.3",
            "sleep @100"
    };

    public String[] presetActionsDetectMoveLeftOnly = {
            //"wheel_left @11 @0.3 @nowait",
            "wheel_left @11 @0.3",
            "sleep @100"
    };

    public String[] presetActionsDetectMoveLeft = {
        //"wheel_forward @12 @0.3 @nowait",
        //"wheel_left @11 @0.3 @nowait",
        "wheel_forward @12 @0.3",
        "wheel_left @11 @0.3",
        "sleep @100"
    };
    public String[] presetActionsDetectMoveBackFromLeft = {
        "wheel_right @11 @0.3 @nowait",
        "wheel_back @12 @0.3 @nowait",
    };
}
