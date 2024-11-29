package org.firstinspires.ftc.teamcode;

public class RobotDataSwiper extends RobotDataBase {

    public RobotDataSwiper (boolean parkingCenter, String workingMode, boolean debugMode) {
        type = "swiper";
        this.parkingCenter = parkingCenter;
        this.workingMode = workingMode;
        this.debugMode = debugMode;

        // for red
        // the 1th/mark0 index is for driving close the team prop
        // the 2th/mark1 index is for driving to the related position to drop the pixel
        // the 3th/mark2 index is for turning left or right (if hadn't been done in the mark1)
        // the 4th/mark3 index is for driving to the backdrop board
        // the 5th/mark4 index is for driving to the related apriltag
        // the 6th/mark5 index is for travel through the tross, need move to the outside path which is close to the operator
        // the 7th/mark6 index is for parking. when select rednear or bluenear, it will park at the center, otherwise park at side;
        markRedNear = new String[][][] {
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markRedLeftIndex1"},
                        {""},
                        {"wheel_back @35 @0.8"}, // driving to the backdrop board
                        {"wheel_right @7 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @22 @0.6" : "wheel_left @38 @0.6", "wheel_back @10 @0.5"},
                }, // left
                {
                        {"wheel_forward @23 @0.3 @nowait"},
                        {"wheel_right @6 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3 @nowait"},
                        {"wheel_back @33 @0.8"},// driving to the backdrop board
                        {"wheel_right @2 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @32 @0.6" : "wheel_left @28 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_right @13 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3 @nowait"},
                        {"wheel_back @28 @0.8"}, // driving to the backdrop board
                        {"wheel_right @6 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @38 @0.6" : "wheel_left @20 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        markRedLeftIndex1 = new String[]{
                "wheel_right @8 @0.2 @nowait",
                "wheel_forward @15 @0.3 @nowait",
                "wheel_turn_left @90 @0.3 @nowait",
                "wheel_forward @3 @0.2 @nowait"
        };

        markRedFar = new String[][][]{
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_left @13 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3"},
                        {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                        {"wheel_right @39 @0.5", "wheel_back @33 @0.8"}, // driving to the apriltag
                        {"wheel_left @11 @0.3"},
                        {parkingCenter ? "wheel_right @21 @0.6" : "wheel_left @33 @0.6", "wheel_back @10 @0.5"},
                }, // left
                {
                        {"wheel_forward @23 @0.3 @nowait"},
                        {"wheel_left @7 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3"},
                        {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                        {"wheel_right @29 @0.5", "wheel_back @28 @0.8"},// driving to the apriltag
                        {"wheel_left @22 @0.4"},
                        {parkingCenter ? "wheel_right @28 @0.6" : "wheel_left @26 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markRedFarRightIndex1"},
                        {"wheel_turn_right @180 @0.3"},
                        {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                        {"wheel_right @18 @0.4", "wheel_back @26 @0.8"},// driving to the apriltag
                        {"wheel_left @30 @0.4"},
                        {parkingCenter ? "wheel_right @34 @0.6" : "wheel_left @19 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        markRedFarRightIndex1 = new String[]{
                "wheel_left @8 @0.3 @nowait",
                "wheel_forward @15 @0.3 @nowait",
                "wheel_turn_right @90 @0.3 @nowait",
                "wheel_forward @3 @0.3 @nowait"
        };

        // for blue
        // (after drop the purple pixel, turn *right*)
        markBlueNear = new String[][][]{
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_left @13 @0.3 @nowait"},
                        {"wheel_turn_right @90 @0.3 @nowait"},
                        {"wheel_back @28 @0.8"}, // driving to the backdrop board
                        {"wheel_left @8 @0.2 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_left @38 @0.6" : "wheel_right @22 @0.6", "wheel_back @10 @0.5"},
                }, // left
                {
                        {"wheel_forward @23 @0.4 @nowait"},
                        {"wheel_left @6 @0.2 @nowait"},
                        {"wheel_turn_right @90 @0.3 @nowait"},
                        {"wheel_back @35 @0.8"}, // driving to the backdrop board
                        {"wheel_left @6 @0.2 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_left @27 @0.6" : "wheel_right @32 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markBlueRightIndex1"},
                        {""},
                        {"wheel_back @36 @0.8"},// driving to the backdrop board
                        {"wheel_left @9 @0.2 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_left @20 @0.6" : "wheel_right @38 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        markBlueRightIndex1 = new String[]{
                "wheel_left @8 @0.2 @nowait",
                "wheel_forward @15 @0.3 @nowait",
                "wheel_turn_right @90 @0.3 @nowait",
                "wheel_forward @3 @0.2 @nowait"
        };

        markBlueFar = new String[][][]{
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markRedLeftIndex1"},
                        {"wheel_turn_right @180 @0.4"},
                        {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long, separate to two actions
                        {"wheel_left @18 @0.4", "wheel_back @28 @0.8"}, // driving to the apriltag
                        {"wheel_right @26 @0.4"},
                        {parkingCenter ? "wheel_left @36 @0.6" : "wheel_right @22 @0.6", "wheel_back @10 @0.5"},
                }, // right
                {
                        {"wheel_forward @23 @0.3"},
                        {"wheel_right @6 @0.2 @nowait"},
                        {"wheel_turn_right @90 @0.3"},
                        {"wheel_back @60 @0.8"}, // driving to the backdrop board
                        {"wheel_left @29 @0.4", "wheel_back @30 @0.8"}, // driving to the apriltag
                        {"wheel_right @20 @0.4"},
                        {parkingCenter ? "wheel_left @26 @0.6" : "wheel_right @30 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_right @13 @0.3 @nowait"},
                        {"wheel_turn_right @90 @0.3"},
                        {"wheel_back @60 @0.8"}, // driving to the backdrop board
                        {"wheel_left @39 @0.4", "wheel_back @36 @0.8"}, // driving to the apriltag
                        {"wheel_right @9 @0.4"},
                        {parkingCenter ? "wheel_left @19 @0.6" : "wheel_right @37 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        // rednear = 0
        // readfar = 1
        // bluenear = 2
        // bluefar = 3
        markDoubleCheckPosition = new String[][] {
                {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // rednear
                {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // redfar
                {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // bluenear
                {"wheel_forward @2 @0.3", "wheel_right @6 @0.3", "sleep @1", "ai_getmarkposition @6", "wheel_left @6 @0.3", "wheel_back @2 @0.3"}, // bluefar
        };
        useDistanceSensor = false;
        useDistanceSensorToDetectMark = true;

        sliderPerInch = 100;
        // high - firstrow = 1500 - 875 = 625 = about 17.25 inch - 11inch = 6.25inch  // per inch = 100
        sliderDefaultPower = 1.0;
        sliderMinPosition = 0;
        sliderMaxPosition = 2200;
        sliderInitialPosition = 0; // may change every time
        sliderFirstRowPosition = 875; // first pixel drop position when no pixel on the board
        sliderHighPosition = 1500;
        sliderSmallStepSize = 4 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
        sliderRow1Position = sliderFirstRowPosition;
        sliderRow2Position = sliderFirstRowPosition + 3 * sliderPerInch;
        sliderRow3Position = sliderFirstRowPosition + 6 * sliderPerInch;
        sliderRow4Position = sliderFirstRowPosition + 9 * sliderPerInch;
        sliderRow5Position = sliderFirstRowPosition + 12 * sliderPerInch;
        sliderLine1Position = sliderRow4Position;
        sliderLine2Position = sliderFirstRowPosition + 18 * sliderPerInch;


        lifterMaxPosition = 1200;

        boxMinPosition = 0.00;
        boxMaxPosition = 1.00;
        boxPickupPosition = 0.60; //0.39; //0.42; //0.49;
        boxPixelSlidePosition = 0.63; //0.55; // pixel may not slide to the bottom of the box, need a pixel slide position
        boxDropPosition = 0.85; //0.86; //0.70; //0.65; //0.72; //0.75 is about vertical to the ground
        boxDropLowPosition = 0.86; //0.87; //0.69; //0.73; // lifter is just one step high
        boxDefaultPosition = boxPickupPosition;

        doorMinPosition = 0.00;
        doorMaxPosition = 1.00;
        doorOpenPosition = 0.30;
        doorClosePosition = 0.50;
        doorDefaultPosition = doorClosePosition; // close

        swiperlifterMinPosition = 0.00;
        swiperlifterMaxPosition = 1.00;
        swiperlifterUpPosition = 0.20;
        swiperlifterDownPosition = 0.80; // ground
        swiperlifterHigh5Position = 0.7;
        swiperlifterHigh4Position = 0.6;
        swiperlifterHigh3Position = 0.5;
        swiperlifterHigh2Position = 0.4;
        swiperlifterHigh1Position = 0.1355;
        swiperlifterDefaultPosition = swiperlifterUpPosition;

        lifterarmMinPosition = 0.00;
        lifterarmMaxPosition = 1.00;
        lifterarmUpPosition = 0.71; // near the slider and ready to drop the pixel
        lifterarmDownPosition = 0.1195; // ground
        lifterarmDefaultPosition = 0.5;

        droneMinPosition = 0.00;
        droneMaxPosition = 1.00;
        droneOpenPosition = 0.34;
        droneClosePosition = 0.56;
        droneDefaultPosition = droneClosePosition;

        presetActionsAutoModeInitBeforeStart = new String[]{
                //"box @pickup"
        };

        presetActionsManualModeInitBeforeStart = new String[]{
                //"box @pickup"
        };

        presetActionsInitAfterStartNonAutoMode = new String[]{
                //"arm @moveready",
                //"drone @close"
        };

        presetActionsLeft = new String[]{
                "sleep @1"
        };

        presetActionsDropPurpleOnly = new String[]{
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

        presetActionsRedNear = new String[]{
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
                "slider @firstrow",
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

        presetActionsRedFar = presetActionsRedNear;

        presetActionsBlueNear = presetActionsRedNear;

        presetActionsBlueFar = presetActionsRedNear;

        presetActionsGetWhitePixels = new String[]{
                "wheel_forward @108 @1.0",
                "getpixel @2",
                "wheel_back @8 @0.4",
                "wheel_turn_right @16.4 @0.4",
                "wheel_forward @100 @1.0",
                "drop_pixel",
                "arm_down",
                "drop_pixel",
                "arm_down",
                "wheel_turn_right @16.4 @0.4",
        };

        presetActionsStep2_left = new String[]{
                "wheel_right @31 @0.2",
                //"elbow_up @2 @0.2"
        };

        presetActionsStep2_right = new String[]{
                "wheel_left @30.5 @0.25",
                //"elbow_up @2 @0.2"
        };

        presetActionsAutoE2E = new String[]{
        };

        presetActionsPad1Up = new String[]{
                "lifter @1.0"
        };

        presetActionsPad1Down = new String[]{
                "lifter @-1.0"
        };

        presetActionsPad1Left = new String[]{
                "lifter @0.0"
        };

        presetActionsPad1Right = new String[]{
                //"transportation @-0.5"
        };

        presetActionsPad1X = new String[]{
                "wheel_left @4 @0.3"
        };
        presetActionsPad1Y = new String[]{
                "wheel_forward @12 @0.2"
        };

        presetActionsPad1A = new String[]{
                "wheel_back @6 @0.2"
        };

        presetActionsPad1B = new String[]{
                "wheel_right @2 @0.3"
        };

        presetActionsPad1UpWithRightBumper = new String[]{
                "nextstep @mark0",
                "nextstep @mark1",
        };
        presetActionsPad1DownWithRightBumper = new String[]{
                "nextstep @presetActionsRedNear"
        };
        presetActionsPad1LeftWithRightBumper = new String[]{
                "grip @close",
                "door @close",
                "ai_getmarkposition",
                "sleep @200",
                "box @pickup",
                "arm @vertical",
        };
        presetActionsPad1RightWithRightBumper = new String[]{
                "arm @down",
                "sleep @600",
                "grip @small",
                "sleep @200",
                "arm @moveready",
                "sleep @300",
                "grip @close",
        };
        presetActionsPad1UpWithLeftBumper = new String[]{
                "slider @firstrow",
                "sleep @300",
                "box @droplow",
        };
        presetActionsPad1DownWithLeftBumper = new String[]{
                "wheel_forward @3 @0.2",
                "box @pickup",
                "slider @min",
                "arm @moveready",
                "nextstep @mark6",
        };
        presetActionsPad1LeftWithLeftBumper = new String[]{
                "nextstep @mark2",
                "nextstep @mark5",
                "sleep @100",
                "nextstep @mark3",
                "nextstep @mark4",
                "balance @redfar @bluefar",
                "distanceslider @0.1 @0.2 @1500",
                "arm @vertical",
        };
        presetActionsPad1RightWithLeftBumper = new String[]{
                "door @open",
                "sleep @1000",
                "door @close",
        };
        presetActionsPad1XWithLeftBumper = new String[]{
                "motor @frontleft @0.3"
                //"ai_getmarkposition"
                //"drone @0.01"
        };
        presetActionsPad1YWithLeftBumper = new String[]{
                "motor @frontright @0.3"
                //"ai_doublecheckmarkposition"
                //"drone @-0.01"
        };
        presetActionsPad1AWithLeftBumper = new String[]{
                "motor @rearleft @0.3"
                //"drone @close"
        };
        presetActionsPad1BWithLeftBumper = new String[]{
                "motor @rearright @0.3"
                //"drone @open"
        };
        presetActionsPad1XWithRightBumper = new String[]{
                "wheel_turn_left @90 @0.3"
        };
        presetActionsPad1YWithRightBumper = new String[]{
                "wheel_turn_left @180 @0.3"
        };
        presetActionsPad1AWithRightBumper = new String[]{
                "wheel_turn_right @90 @0.4"
        };
        presetActionsPad1BWithRightBumper = new String[]{
                "wheel_turn_right @180 @0.4"
        };

        presetActionsWheelTurnLeft = new String[]{
                "wheel_turn_left @90 @0.2"
        };

        presetActionsWheelTurnRight = new String[]{
                "wheel_turn_right @90 @0.2"
        };

        presetActionsPad1Back = new String[]{
                "drone @open"
        };

        presetActionsPad1Start = new String[]{
                "log"
        };

        presetActionsPad2Up = new String[]{
                "slider @up",
                //"box @drop"
        };
        presetActionsPad2Down = new String[]{
                "slider @down",
                //"box @pickup"
        };
        presetActionsPad2Left = new String[]{
                //"transportation @0.5"
        };
        presetActionsPad2Right = new String[]{
                //"transportation @-0.5"
        };

        presetActionsPad2UpWithLeftBumper = new String[]{
                "slider @0.3"
        };
        presetActionsPad2DownWithLeftBumper = new String[]{
                "slider @-0.3"
        };
        presetActionsPad2LeftWithLeftBumper = new String[]{
                "swiperlifter @0.01"
        };
        presetActionsPad2RightWithLeftBumper = new String[]{
                "swiperlifter @-0.01"
        };

        presetActionsPad2UpWithRightBumper = new String[]{
                "slider @max"
        };
        presetActionsPad2DownWithRightBumper = new String[]{
                "slider @min"
        };
        presetActionsPad2LeftWithRightBumper = new String[]{
                "slider @high2"
        };
        presetActionsPad2RightWithRightBumper = new String[]{
                "slider @high3"
        };
        presetActionsPad2X = new String[]{
                "swiper @-0.7",
                //"transportation @-0.5"
        };
        presetActionsPad2Y = new String[]{
                "swiper @1.0"
        };
        presetActionsPad2A = new String[]{
                //"transportation @0.0",
                "swiper @0.0",
                "slider @0.0",
        };
        presetActionsPad2B = new String[]{
                "swiper @0.7"
        };

        presetActionsPad2XWithRightBumper = new String[]{
                "drone @1.0"
        };
        presetActionsPad2YWithRightBumper = new String[]{
                "lifterarm @0.01"
        };
        presetActionsPad2AWithRightBumper = new String[]{
                "lifterarm @-0.01"
        };
        presetActionsPad2BWithRightBumper = new String[]{
                "drone @0.0"
        };
        presetActionsPad2XWithLeftBumper = new String[]{
                "grip @0.01"
        };
        presetActionsPad2YWithLeftBumper = new String[]{
                "lifterarm @pos @0.50"
        };
        presetActionsPad2AWithLeftBumper = new String[]{
                "lifterarm @pos @0.20"
        };
        presetActionsPad2BWithLeftBumper = new String[]{
                "grip @-0.01"
        };
        presetActionsPad2XWithLeftStickX = new String[]{
                "swiperlifter @high3"
        };
        presetActionsPad2YWithLeftStickX = new String[]{
                "swiperlifter @up"
        };
        presetActionsPad2AWithLeftStickX = new String[]{
                "swiperlifter @down"
        };
        presetActionsPad2BWithLeftStickX = new String[]{
                "swiperlifter @high4"
        };
        presetActionsPad2LeftStick = new String[]{
                //"transportation @0.0",
                "swiper @0.0",
                "slider @0.0",
        };
        presetActionsPad2RightStick = new String[]{
                "log"
        };
        presetActionsPad2LeftTrigger = new String[]{
                "lifterarm @0.01",
        };
        presetActionsPad2RightTrigger = new String[]{
                "door @open",
                "sleep @200",
                "door @close"
        };
        presetActionsPad2Back = new String[]{
                "log"
        };
        presetActionsPad2Start = new String[]{
                "arm @vertical"
        };
        presetActionsDefault = new String[]{
                "sleep @1"
        };
        presetActionsPad2LeftstickRightStick = new String[]{
                "drone @1"
        };

        presetActionsDetectMoveRight = new String[]{
                "wheel_right @11 @0.3",
                "sleep @500"
        };
        presetActionsDetectMoveLeft = new String[]{
                "wheel_left @11 @0.3"
        };
    }
}
