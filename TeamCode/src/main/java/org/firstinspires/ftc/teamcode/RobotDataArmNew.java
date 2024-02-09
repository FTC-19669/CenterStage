package org.firstinspires.ftc.teamcode;

public class RobotDataArmNew extends RobotDataBase {

    public RobotDataArmNew(boolean parkingCenter, String workingMode, boolean debugMode) {

        type = "armnew";
        this.parkingCenter = parkingCenter;
        this.workingMode = workingMode;
        this.useDistanceSensor = true; // only for debug
        this.debugMode = debugMode;

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

        // grip data
        perStepSizeGrip = 0.01;
        gripMinPosition = 0.00;
        gripMaxPosition = 1.00;
        gripOpenSmallPosition = 0.7;
        gripOpenMiddlePosition = 0.75;
        gripOpenLargePosition = 0.8;
        gripClosePosition = 0.55;
        gripDefaultPosition = gripClosePosition; // close, old value is 0.60

        grip2MinPosition = 0.00;
        grip2MaxPosition = 1.00;
        grip2OpenSmallPosition = 0.36;
        grip2OpenLargePosition = 0.59;
        grip2ClosePosition = 0.31;
        grip2DefaultPosition = gripClosePosition; // close, old value is 0.60

        armMinPosition = 0.00;
        armMaxPosition = 1.00;
        armUpPosition = 0.12; // near the slider and ready to drop the pixel
        armVerticalPosition = 0.47;
        armDownPosition = 0.86; // ground
        armHigh5Position = 0.77;
        armHigh4Position = 0.79; //armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 3;
        armHigh3Position = armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 2;
        armHigh2Position = armDownPosition + ((armHigh5Position - armDownPosition) / 4) * 1;
        armHigh1Position = armDownPosition;
        armMoveReadyPosition = 0.19;
        armDefaultPosition = armMoveReadyPosition; //0.66; // close to the slider but not so close

        boxMinPosition = 0.00;
        boxMaxPosition = 1.00;
        boxPickupPosition = 0.54;
        boxPixelSlidePosition = 0.55; // pixel may not slide to the bottom of the box, need a pixel slide position
        boxDropPosition = 0.76;  //is about vertical to the ground
        boxDropLowPosition= 0.76; //0.86; //0.87; //0.69; //0.73; // lifter is just one step high
        boxDefaultPosition = boxPickupPosition;

        doorMinPosition = 0.00;
        doorMaxPosition = 1.00;
        doorOpenPosition = 0.19;
        doorClosePosition = 0.51;
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

        distanceTurn90Degree = 24.35; //15.655;

        presetActionsAutoModeInitBeforeStart = new String[]{
                //"box @pickup"
        };
        presetActionsManualModeInitBeforeStart = new String[]{
                "box @pickup"
        };

        presetActionsInitAfterStartNonAutoMode = new String[]{
                //"arm @moveready",
                //"drone @close"
        };
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
                {parkingCenter ? "wheel_right @22 @0.6": "wheel_left @36 @0.6", "wheel_back @10 @0.5"},
            }, // left
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_right @6 @0.3 @nowait"},
                {"wheel_turn_left @90 @1.0"},
                {"wheel_back @33 @0.8 @nowait"},// driving to the backdrop board
                {"wheel_right @2 @0.3 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_right @31 @0.8": "wheel_left @28 @0.6", (parkingCenter && getWhitePixels) ? "nextstep @presetActionsGetWhitePixels" : "wheel_back @10 @0.5"},
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_right @13 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3 @nowait"},
                {"wheel_back @28 @0.8"}, // driving to the backdrop board
                {"wheel_right @8 @0.3 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_right @38 @0.6": "wheel_left @20 @0.6", (parkingCenter && getWhitePixels) ? "nextstep @presetActionsGetWhitePixels" : "wheel_back @10 @0.5"},
            }, // right
        };

        String[][][] markRedNearBak = new String[][][] {
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markRedLeftIndex1"},
                        {""},
                        {"wheel_back @35 @0.8"}, // driving to the backdrop board
                        {"wheel_right @7 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @22 @0.6": "wheel_left @38 @0.6", "wheel_back @10 @0.5"},
                }, // left
                {
                        {"wheel_forward @23 @0.3 @nowait"},
                        {"wheel_right @6 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3 @nowait"},
                        {"wheel_back @33 @0.8"},// driving to the backdrop board
                        {"wheel_right @2 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @32 @0.6": "wheel_left @28 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_right @13 @0.3 @nowait"},
                        {"wheel_turn_left @90 @0.3 @nowait"},
                        {"wheel_back @28 @0.8"}, // driving to the backdrop board
                        {"wheel_right @6 @0.3 @nowait"}, // driving to the apriltag
                        {""},
                        {parkingCenter ? "wheel_right @38 @0.6": "wheel_left @20 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        markRedLeftIndex1 = new String[] {
            "wheel_right @8 @0.2 @nowait",
            "wheel_forward @13 @0.3 @nowait",
            "wheel_turn_left @90 @0.3 @nowait",
            "wheel_forward @3 @0.2 @nowait"
        };

        presetActionsGetOnePixelFromStack = new String[]{
                // arm and grip should be already in position
                //"grip @open",
                //"arm @high5",
                "grip @close",
                "sleep @500",
                "wheel_back @3 @0.3 @nowait",
                "arm @up",
                "sleep @1000",
                "grip @middle",
                "sleep @500",
                "grip @small",
                "arm @moveready",
        };

        // note: the arm position is different from the get one pixel.
        // to get two pixels, the arm should be lower, and it should be done before calling this
        presetActionsGetTwoPixelFromStack = presetActionsGetOnePixelFromStack;

        markRedFar = new String[][][] {
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_left @13 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3", "wheel_right @13 @0.3 @nowait", "wheel_forward @7 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @4 @0.3 @nowait", "wheel_left @13 @0.3 @nowait"},
                {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long, separate to two actions
                {"wheel_right @39 @0.8", "wheel_back @34 @0.8"}, // driving to the apriltag
                {"wheel_left @11 @0.3"},
                {parkingCenter ? "wheel_right @21 @0.6 @nowait" : "wheel_left @33 @0.6 @nowait", "wheel_back @10 @0.5", "slider @min"},
            }, // left
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_left @7 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3", "wheel_right @2 @0.3 @nowait", "wheel_forward @11.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @8.5 @0.3 @nowait", "wheel_left @2 @0.3 @nowait"},
                {"wheel_back @64 @0.8"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                {"wheel_right @29 @0.8", "wheel_back @29 @0.8"},// driving to the apriltag
                {"wheel_left @22 @0.4"},
                {parkingCenter ? "wheel_right @28 @0.6" :"wheel_left @27 @0.6", "wheel_back @10 @0.5", "slider @min"},
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"nextstep @markRedFarRightIndex1"},
                {"sleep @200", "wheel_turn_right @180 @0.8", "wheel_forward @14.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @11.5 @0.3 @nowait",},
                {"wheel_back @63 @1.0 @nowait"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                {"wheel_right @18 @0.4", "wheel_back @25 @0.8 @nowait"},// driving to the apriltag
                {"wheel_left @29 @0.4"},
                {parkingCenter ? "wheel_right @34 @0.8 @nowait" :"wheel_left @19 @0.6 @nowait", "wheel_back @10 @0.5"},
            }, // right
        };

        String[][][] markRedFarBak = new String[][][] {
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
                        {parkingCenter ? "wheel_right @28 @0.6" :"wheel_left @26 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"nextstep @markRedFarRightIndex1"},
                        {"wheel_turn_right @180 @0.3"},
                        {"wheel_back @63 @0.8"}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                        {"wheel_right @18 @0.4", "wheel_back @26 @0.8"},// driving to the apriltag
                        {"wheel_left @30 @0.4"},
                        {parkingCenter ? "wheel_right @34 @0.6" :"wheel_left @19 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        markRedFarRightIndex1 = new String[] {
            "wheel_left @8 @0.3 @nowait",
            "wheel_forward @14 @0.3 @nowait",
            "wheel_turn_right @90 @0.3 @nowait",
            "wheel_forward @3 @0.3 @nowait"
        };

        // for blue
        // (after drop the purple pixel, turn *right*)
        markBlueNear = new String[][][] {
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_left @13 @0.3 @nowait"},
                {"wheel_turn_right @90 @0.3 @nowait"},
                {"wheel_back @28 @0.8"}, // driving to the backdrop board
                {"wheel_left @8 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @38 @0.6" : "wheel_right @22 @0.6", "wheel_back @10 @0.5" },
            }, // left
            {
                {"wheel_forward @23 @0.4 @nowait"},
                {"wheel_left @6 @0.2 @nowait"},
                {"wheel_turn_right @90 @0.3 @nowait"},
                {"wheel_back @35 @0.8"}, // driving to the backdrop board
                {"wheel_left @6 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @27 @0.6" : "wheel_right @32 @0.6", "wheel_back @10 @0.5" },
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"nextstep @markBlueRightIndex1"},
                {""},
                {"wheel_back @36 @0.8"},// driving to the backdrop board
                {"wheel_left @9 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @20 @0.6" : "wheel_right @38 @0.6", "wheel_back @10 @0.5" },
            }, // right
        };

        markBlueRightIndex1 = new String[] {
            "wheel_left @8 @0.3 @nowait",
            "wheel_forward @15 @0.3 @nowait",
            "wheel_turn_right @90 @0.3 @nowait",
            "wheel_forward @3 @0.3 @nowait"
        };

        markBlueFar = new String[][][] {
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"nextstep @markRedLeftIndex1"},
                {"sleep @200", "wheel_turn_right @180 @0.8", "wheel_forward @14.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @11.5 @0.3 @nowait",},
                {"wheel_back @64 @1.0"}, // driving to the backdrop board, originally the distance is too long, separate to two actions
                {"wheel_left @18 @0.4", "wheel_back @27 @0.8 @nowait"}, // driving to the apriltag
                {"wheel_right @26 @0.4"},
                {parkingCenter ? "wheel_left @36 @0.8 @nowait" : "wheel_right @22 @0.6 @nowait", "wheel_back @10 @0.5"},
            }, // right
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_right @6 @0.3 @nowait"},
                {"wheel_turn_right @90 @0.8", "wheel_left @2 @0.3 @nowait", "wheel_forward @11.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @8.5 @0.3 @nowait", "wheel_right @2 @0.3 @nowait"},
                {"wheel_back @60 @1.0"}, // driving to the backdrop board
                {"wheel_left @29 @0.8", "wheel_back @30 @0.8"}, // driving to the apriltag
                {"wheel_right @20 @0.4"},
                {parkingCenter ? "wheel_left @26 @0.6" :"wheel_right @30 @0.6", "wheel_back @10 @0.5", "slider @min"},
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_right @13 @0.3 @nowait"},
                {"wheel_turn_right @90 @0.3", "wheel_left @13 @0.3 @nowait", "wheel_forward @7 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @4 @0.3 @nowait", "wheel_right @13 @0.3 @nowait"},
                {"wheel_back @60 @0.8"}, // driving to the backdrop board
                {"wheel_left @39 @0.8", "wheel_back @36 @0.8"}, // driving to the apriltag
                {"wheel_right @9 @0.4"},
                {parkingCenter ? "wheel_left @19 @0.6 @nowait" :"wheel_right @37 @0.6 @nowait", "wheel_back @10 @0.5", "slider @min"},
            }, // right
        };

        String[][][] markBlueFarBak = new String[][][] {
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
                        {parkingCenter ? "wheel_left @26 @0.6" :"wheel_right @30 @0.6", "wheel_back @10 @0.5"},
                }, // center
                {
                        {"wheel_forward @12 @0.3 @nowait"},
                        {"wheel_right @13 @0.3 @nowait"},
                        {"wheel_turn_right @90 @0.3"},
                        {"wheel_back @60 @0.8"}, // driving to the backdrop board
                        {"wheel_left @39 @0.4", "wheel_back @36 @0.8"}, // driving to the apriltag
                        {"wheel_right @9 @0.4"},
                        {parkingCenter ? "wheel_left @19 @0.6" :"wheel_right @37 @0.6", "wheel_back @10 @0.5"},
                }, // right
        };

        presetActionsDropPurpleOnly = new String[]{
            "grip @close",
            "door @close",
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            "ai_getmarkposition @5",
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
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            "ai_getmarkposition @5",
            "nextstep @mark0",
            "nextstep @mark1",
            "recordposition @stop",
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
            "balance",
            "arm @vertical",
            workingMode.contains("near") ? "slider @firstrow": "slider @high3",
            //"slider @firstrow",
            "sleep @300",
            "box @droplow",
            //"sleep @1000",
            "distanceslider @0.1 @0.2 @1500",
            "balance @reset",
            "door @open",
            "sleep @1000", //"sleep @500",
            "wheel_forward @3 @0.3 @nowait",
            "box @pickup",
            "slider @min",
            "door @close",
            (parkingCenter && getWhitePixels) ?  "arm @high4" : "arm @moveready",
            (parkingCenter && getWhitePixels) ?  "grip @open" : "grip @small",
            "nextstep @mark6",
            //"checktimeleft @16000", //at least have these milliseconds left, then do the next step
            //"nextstep @presetActionsGetPixels",
        };

        presetActionsRedFar = new String[]{
                "grip @close",
                "door @close",
                "box @pickup",
                "sleep @200",
                "arm @vertical",
                "ai_getmarkposition @5",
                "nextstep @mark0",
                "nextstep @mark1",
                "recordposition @stop",
                "arm @down",
                "sleep @600",
                "grip @small",
                "sleep @200",
                "arm @high5",
                //"sleep @200",
                "grip @open",
                "nextstep @mark2",
                "nextstep @mark5",
                "sleep @100",
                "nextstep @mark3",
                "nextstep @mark4",
                //"balance @redfar @bluefar",
                "arm @vertical",
                workingMode.contains("near") ? "slider @firstrow": "slider @high2",
                "sleep @300",
                "box @droplow",
                //"sleep @1000",
                "distanceslider @0.1 @0.2 @1500",
                "door @open",
                "sleep @500",
                "wheel_forward @3 @0.3 @nowait",
                "box @pickup",
                "slider @min",
                "door @close",
                "arm @moveready",
                "nextstep @mark6",
        };

        presetActionsBlueNear = presetActionsRedNear;

        presetActionsBlueFar = presetActionsRedFar;

        // this get pixel script is under testing, not finished yet
        presetActionsGetWhitePixels = new String[] {
                "balance",
                "wheel_forward @98 @1.0",
                "wheel_forward @5 @0.3",
                "nextstep @presetActionsGetTwoPixelFromStack",
                "wheel_back @100 @1.0",
                "wheel_left @23 @0.6 @nowait",
                "arm @vertical",
                "slider @high3",
                //"slider @firstrow",
                "sleep @300",
                "box @droplow",
                //"sleep @1000",
                "distanceslider @0.1 @0.2 @1500",
                "door @open",
                "sleep @500",
                "wheel_forward @3 @0.3 @nowait",
                "box @pickup",
                "slider @min",
                "door @close",
                "arm @moveready",
                "wheel_right @20 @0.6 @nowait",
                "",
        };

        presetActionsDetectMoveRight = new String[]{
            "wheel_forward @12 @0.3 @nowait",
            "wheel_right @13 @0.3 @nowait",
            "sleep @100"
        };
        presetActionsDetectMoveBackFromRight = new String[]{
            "wheel_left @13 @0.3 @nowait",
            "wheel_back @12 @0.3 @nowait",
        };
        presetActionsDetectMoveLeft = new String[]{
            "wheel_forward @12 @0.3 @nowait",
            "wheel_left @13 @0.3 @nowait",
            "sleep @100"
        };
        presetActionsDetectMoveBackFromLeft = new String[]{
            "wheel_right @13 @0.3 @nowait",
            "wheel_back @12 @0.3 @nowait",
        };

        presetActionsPad1Up = new String[]{
                "distanceslider @0.0 @0.3 @1000"
        };

        presetActionsPad1Down = new String[]{
                "distanceslider @0.1 @0.3 @1000"
        };

        presetActionsPad1Left = new String[]{
                //"distancegrip @2 @0.3"
        };

        presetActionsPad1Right = new String[]{
                //"distancegrip @3 @0.3"
        };

        presetActionsPad1X = new String[]{
                //"lifter @0.4"
        };
        presetActionsPad1Y = new String[]{
                "arm @vertical"
        };

        presetActionsPad1A = new String[]{
                "arm @moveready"
        };

        presetActionsPad1B = new String[]{
                //"lifter @-0.4"
        };

        presetActionsPad1UpWithRightBumper = new String[]{
                "nextstep @mark0",
                "nextstep @mark1",
                "recordposition @stop",
        };
        presetActionsPad1DownWithRightBumper = new String[]{
                "nextstep @presetActionsRedNear"
        };
        presetActionsPad1LeftWithRightBumper = new String[]{
                "grip @close",
                "door @close",
                "box @pickup",
                "sleep @200",
                "arm @vertical",
                "ai_getmarkposition @5",
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
        presetActionsPad1DownWithLeftBumper = new String[]{
                "wheel_forward @3 @0.2",
                "box @pickup",
                "slider @min",
                "arm @moveready",
                "nextstep @mark6",
        };
        presetActionsPad1LeftWithLeftBumper = new String[]{
                "nextstep @mark2",
        };
        presetActionsPad1RightWithLeftBumper = new String[]{
                "door @open",
                "sleep @1000",
                "door @close",
        };
        presetActionsPad1XWithLeftBumper = new String[]{
                //"lifter @0.8",
                //"sleep @3000",
                //"lifter @0.2",
                "lifter @-0.5",
        };
        presetActionsPad1YWithLeftBumper = new String[]{
                //"lifter @-0.4",
                //"lifterhelper @1.0",
                //"sleep @11000",
                //"lifter @0.0",
                //"lifterhelper @0.0"
                "lifter @-1.0",
        };
        presetActionsPad1AWithLeftBumper = new String[]{
                "lifterhelper @0.0",
                "lifter @0.0"
        };
        presetActionsPad1BWithLeftBumper = new String[]{
                //"door @open",
                //"sleep @150",
                //"door @close",
                "lifterhelper @1.0 @5000"
        };
        presetActionsPad1XWithRightBumper = new String[]{
                //"wheel_turn_left @90 @0.3"
                //"lifter @0.8"
        };
        presetActionsPad1YWithRightBumper = new String[]{
                //"wheel_turn_left @180 @0.3"
                "lifter @-0.5",
                "lifterhelper @-1.0",
                "sleep @1000",
                "lifter @0.0",
                "lifterhelper @0.0"
        };
        presetActionsPad1AWithRightBumper = new String[]{
                //"wheel_turn_right @90 @0.4"
                "lifter @0.8",
                "sleep @3000",
                "lifter @0.2",
        };
        presetActionsPad1BWithRightBumper = new String[]{
                //"wheel_turn_right @180 @0.4"
                //"lifter @-0.8"
                "lifter @0.8",
                "sleep @1000",
                "lifter @0.2",
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

        presetActionsPad1Start = new String[] {
                "log"
        };

        presetActionsPad2Up = new String[]{
                "arm @vertical",
                "slider @up",
                "box @drop"
        };
        presetActionsPad2Down = new String[]{
                "slider @down",
                "box @pickup"
        };
        presetActionsPad2Left = new String[]{
                "box @pickup"
        };
        presetActionsPad2Right = new String[]{
                "box @drop"
        };

        presetActionsPad2UpWithLeftBumper = new String[]{
                "slider @0.8"
        };
        presetActionsPad2DownWithLeftBumper = new String[]{
                "slider @-0.8"
        };
        presetActionsPad2LeftWithLeftBumper = new String[]{
                "box @0.01"
                //"slider @0.0"
        };
        presetActionsPad2RightWithLeftBumper = new String[]{
                "box @-0.01"
                //"slider @0.0"
        };

        presetActionsPad2UpWithRightBumper = new String[]{
                "arm @vertical",
                "slider @up",
                "box @drop",
                "slider @max"
        };
        presetActionsPad2DownWithRightBumper = new String[]{
                "slider @down",
                "box @pickup",
                "slider @min",
                "arm @moveready"
        };
        presetActionsPad2LeftWithRightBumper = new String[]{
                "slider @high2"
        };
        presetActionsPad2RightWithRightBumper = new String[]{
                "slider @high3"
        };

        presetActionsPad2X = new String[]{
                "grip @large"
        };
        presetActionsPad2Y = new String[]{
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
        presetActionsPad2A = new String[]{
                "grip @close",
                "sleep @100",
                "arm @down",
                "sleep @300",
                "grip @large"
        };
        presetActionsPad2B = new String[]{
                "grip @close"
        };

        presetActionsPad2XWithRightBumper = new String[]{
                "door @0.01"
        };

        presetActionsPad2YWithRightBumper = new String[]{
                "grip @small"
        };
        presetActionsPad2AWithRightBumper = new String[]{
                "door @close"
        };
        presetActionsPad2BWithRightBumper = new String[]{
                "door @-0.01"
        };
        presetActionsPad2XWithLeftBumper = new String[]{
                "grip @0.01"
        };
        presetActionsPad2YWithLeftBumper = new String[]{
                "arm @0.01"
        };
        presetActionsPad2AWithLeftBumper = new String[]{
                "arm @-0.01"
        };
        presetActionsPad2BWithLeftBumper = new String[]{
                "grip @-0.01"
        };
        presetActionsPad2XWithLeftStickX = new String[]{
                "arm @high3"
        };
        presetActionsPad2YWithLeftStickX = new String[]{
                "arm @moveready",
        };
        presetActionsPad2AWithLeftStickX = new String[]{
                "arm @vertical",
        };
        presetActionsPad2BWithLeftStickX = new String[]{
                "arm @high4"
        };
        presetActionsPad2LeftStick = new String[]{
                "arm @moveready",
        };
        presetActionsPad2RightStick = new String[]{
                "arm @vertical",
        };
        presetActionsPad2LeftTrigger = new String[]{
                "door @open",
        };
        presetActionsPad2RightTrigger = new String[]{
                "door @open",
                "sleep @800",
                "door @close",
        };
        presetActionsPad2Back = new String[]{
                "log"
        };
        presetActionsPad2Start = new String[]{
                "arm @vertical"
        };
        presetActionsDefault = new String[]{
                "sleep @100"
        };
        presetActionsPad2LeftstickRightStick = new String[]{
                "drone @1"
        };
    }
}
