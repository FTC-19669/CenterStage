package org.firstinspires.ftc.teamcode;

public class RobotDataArm extends RobotDataBase {

    public RobotDataArm (boolean parkingCenter, String workingMode, boolean debugMode, boolean hangFarSide) {

        type = "arm";
        this.parkingCenter = parkingCenter;
        this.workingMode = workingMode;
        this.debugMode = debugMode;
        this.hangFarSide = hangFarSide;

        useLifter = true;

        sliderPerInch = 100;
        // high - firstrow = 1500 - 875 = 625 = about 17.25 inch - 11inch = 6.25inch  // per inch = 100
        sliderDefaultPower = 1.0;
        sliderMinPosition = 0;
        sliderMaxPosition = 2200;
        sliderInitialPosition = 0; // may change every time
        sliderFirstRowPosition = 875; // first pixel drop position when no pixel on the board
        sliderHighPosition = 1500;
        sliderStepSize = 5 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
        sliderSmallStepSize = 3 * sliderPerInch; // a litter bigger than 1 pixel height (3 inches)
        sliderDownStepSize = 2 * sliderPerInch; // less than the up value as the slider will go down by itself
        sliderRow0Position = sliderFirstRowPosition - (sliderRow0PositionLower ? 3 : 1) * sliderPerInch; // a little lower than one pixel height, no way to jump
        sliderRow1Position = sliderFirstRowPosition + sliderPerInch;
        sliderRow1FarPosition = sliderFirstRowPosition; //sliderFirstRowPosition - 1 * sliderPerInch;
        sliderRow2Position = sliderFirstRowPosition + 3 * sliderPerInch;
        sliderRow3Position = sliderFirstRowPosition + 6 * sliderPerInch;
        sliderRow4Position = sliderFirstRowPosition + 9 * sliderPerInch;
        sliderRow5Position = sliderFirstRowPosition + 12 * sliderPerInch;
        sliderLine1Position = sliderRow4Position;
        sliderLine2Position = sliderFirstRowPosition + 18 * sliderPerInch;

        lifterMaxPosition = 1200;

        if (useArm1) {
            // grip data
            perStepSizeGrip = 0.01;
            gripMinPosition = 0.00;
            gripMaxPosition = 1.00;
            gripOpenSmallPosition = 0.46; //0.49; //0.52; // old value is 0.53;
            gripOpenMiddlePosition = 0.44;
            gripOpenLargePosition = 0.34;
            gripOpenVeryLargePosition = 0.22;
            gripClosePosition = 0.56; //0.53; //0.59;
            gripDefaultPosition = gripClosePosition; // close, old value is 0.60

            armMinPosition = 0.00;
            armMaxPosition = 1.00;
            armUpPosition = 0.33; //0.05;// 0.18; //0.20; //0.05; // near the slider and ready to drop the pixel
            armVerticalPosition = 0.60; //0.31; //0.44;
            armDownPosition = 0.99; //0.71; //0.84;
            armHigh5Position = 0.94; //0.663; //0.790; //0.783; //0.640; //0.647; // 0.645; the bigger the lower
            armHigh4Position = 0.94;
            armHigh3Position = 0.96;
            armHigh2Position = 0.98;
            armHigh1Position = armDownPosition;
            armFirstRowPosition = 0.84; //0.57; //0.70;
            armSecondRowPosition = 0.79; //0.52; //0.65;
            armMoveReadyPosition = 0.37; //0.12; //0.24;
            armDefaultPosition = armMoveReadyPosition; //0.66; // close to the slider but not so close
        }
        else {
            // grip data
            perStepSizeGrip = 0.01;
            gripMinPosition = 0.00;
            gripMaxPosition = 1.00;
            gripOpenSmallPosition = 0.68; //0.41;//0.46; //0.49; //0.52; // old value is 0.53;
            gripOpenMiddlePosition = 0.64; //0.39; // 0.44;
            gripOpenLargePosition = 0.60;//0.29; //0.34;
            gripOpenVeryLargePosition = 0.46; //0.20;//0.22;
            gripClosePosition = 0.76; //0.51; //0.56; //0.53; //0.59;
            gripDefaultPosition = gripClosePosition; // close, old value is 0.60

            armMinPosition = 0.00;
            armMaxPosition = 1.00;
            armUpPosition = 0.21; //0.18; //0.20; //0.05; // near the slider and ready to drop the pixel
            armVerticalPosition = 0.47; //0.44;
            armDownPosition = 0.87; //0.84;
            armHigh5Position = 0.83; //0.790; //0.783; //0.640; //0.647; // 0.645; the bigger the lower
            armHigh4Position = 0.67;
            armHigh3Position = 0.68;
            armHigh2Position = 0.69;
            armHigh1Position = armDownPosition;
            armFirstRowPosition = 0.73; //0.70;
            armSecondRowPosition = 0.68; //0.65;
            armMoveReadyPosition = 0.28; //0.24;
            armDefaultPosition = armMoveReadyPosition; //0.66; // close to the slider but not so close
        }

        if (useBox1) {
            boxMinPosition = 0.00;
            boxMaxPosition = 1.00;
            boxPickupPosition = 0.70; //0.63; //0.66; //0.68; //0.61; //0.58; // 0.60; //0.39; //0.42; //0.49;
            boxPixelSlidePosition = 0.70; //0.63; //0.66; //0.64; //0.61; //0.63; //0.55; // pixel may not slide to the bottom of the box, need a pixel slide position
            boxDropPosition = 0.88; //0.86; //0.83; //0.85; //0.86; //0.70; //0.65; //0.72; //0.75 is about vertical to the ground
            boxDropLowPosition = boxDropPosition; //0.88; //0.91; //0.86; //0.87; //0.69; //0.73; // lifter is just one step high
            boxDropLowLeftPosition = 0.93; //0.91;
            //boxDropLowPosition = useNewDroplowPosition ? 0.89 : boxDropLowPosition;
            boxDropPosition = useNewDroplowPosition ? boxDropLowLeftPosition : boxDropPosition;
            boxDefaultPosition = boxPickupPosition;

            doorMinPosition = 0.00;
            doorMaxPosition = 1.00;
            doorOpenPosition = 0.11;
            doorOpenOnePosition = 0.17;
            doorClosePosition = 0.03;
            doorDefaultPosition = doorClosePosition; // close
        }
        else {
            boxMinPosition = 0.00;
            boxMaxPosition = 1.00;
            boxPickupPosition = 0.70; //0.63; //0.66; //0.68; //0.61; //0.58; // 0.60; //0.39; //0.42; //0.49;
            boxPixelSlidePosition = 0.70; //0.63; //0.66; //0.64; //0.61; //0.63; //0.55; // pixel may not slide to the bottom of the box, need a pixel slide position
            boxDropPosition = 0.88; //0.86; //0.83; //0.85; //0.86; //0.70; //0.65; //0.72; //0.75 is about vertical to the ground
            boxDropLowPosition= boxDropPosition; //0.88; //0.91; //0.86; //0.87; //0.69; //0.73; // lifter is just one step high
            boxDropLowLeftPosition= 0.93; //0.91;
            //boxDropLowPosition = useNewDroplowPosition ? 0.89 : boxDropLowPosition;
            boxDropPosition = useNewDroplowPosition ? boxDropLowLeftPosition : boxDropPosition;
            boxDefaultPosition = boxPickupPosition;

            doorMinPosition = 0.00;
            doorMaxPosition = 1.00;
            doorOpenPosition = useTwoWayDoor ? 0.10 : 0.30;
            doorOpenOnePosition = useTwoWayDoor ? 0.13 : 0.30;
            doorClosePosition = useTwoWayDoor ? 0.04 : 0.50;
            doorDefaultPosition = doorClosePosition; // close
        }

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
        lifterarmUpPosition = 0.13; // ready for lifter to up
        lifterarmDownPosition = 1.0; // ground
        lifterarmDefaultPosition = lifterarmUpPosition;

        droneMinPosition = 0.00;
        droneMaxPosition = 1.00;
        droneOpenPosition = 0.34;
        droneClosePosition = 0.56;
        droneDefaultPosition = droneClosePosition;

        distanceTurn90Degree = 24.35; //15.655;

        presetActionsAutoModeInitBeforeStart = new String[]{
                //"set @imu",
                //"box @pickup"
        };
        presetActionsManualModeInitBeforeStart = new String[]{
                "box @pickup"
        };

        presetActionsInitAfterStartNonAutoMode = new String[]{
                //"set @imu",
                debugMode ? "exit" : "",
                "grip @close",
                "box @pickup",
                "door @close",
                "arm @moveready",
                //"lifterarm @down",
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
                {parkingCenter ? "wheel_right @23.5 @0.6": "wheel_left @36 @0.6", "wheel_back @11 @0.5"},
            }, // left
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_right @6 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3 @nowait"},
                {"wheel_back @30 @0.8"},// driving to the backdrop board
                {"wheel_right @5 @0.3 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_right @30 @0.8": "wheel_left @29 @0.6", (parkingCenter && getWhitePixels) ? "nextstep @presetActionsGetWhitePixels" : "wheel_back @11 @0.5"},
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_right @12 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3 @nowait"},
                {"wheel_back @28 @0.8"}, // driving to the backdrop board
                {"wheel_right @9 @0.3 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_right @37 @0.6": "wheel_left @22.5 @0.6", (parkingCenter && getWhitePixels) ? "nextstep @presetActionsGetWhitePixels" : "wheel_back @11 @0.5"},
            }, // right
        };

        markRedLeftIndex1 = new String[] {
            "wheel_right @8 @0.3 @nowait",
            "wheel_forward @14 @0.3 @nowait",
            "wheel_turn_left @90 @0.3 @nowait",
            "wheel_forward @3 @0.3 @nowait"
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
                {"wheel_forward @12 @0.3 @nowait"}, //mark0
                {"wheel_left @13.5 @0.3 @nowait"}, //mark1
                // following line the first wheel_forward and wheel_back is to push the team prop away, otherwise it may stuck in the wheels
                {"wheel_forward @18 @0.3 @nowait", "wheel_back @5 @0.3 @nowait", "wheel_turn_left @90 @0.3 @nowait", "wheel_forward @7 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @3 @0.3 @nowait", "wheel_left @13 @0.3 @nowait"}, //mark2
                {"imuturn @92 @0.2", "wheel_back @64 @0.4 @yaw92", "check @imu @92", "imuturn @92 @0.2",}, // mark3, driving to the backdrop board, originally the distance is too long, separate to two actions
                {"sleep @10", "wheel_right @36 @0.4", "wheel_back @32 @0.4 @yaw92"}, // mark4, driving to the apriltag
                {"wheel_left @13 @0.3"}, //mark5
                {parkingCenter ? "wheel_right @19 @0.6 @nowait" : "wheel_left @32 @0.6 @nowait", "wheel_back @10 @0.5", "slider @min"}, //mark6
                {"wheel_forward @2 @0.2 @nowait", "wheel_left @4 @0.2 @nowait", "wheel_back @2 @0.2 @nowait",}, //mark7
            }, // left
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_left @7 @0.3 @nowait"},
                {"wheel_turn_left @90 @0.3 @nowait", "wheel_right @2.5 @0.3 @nowait", "", "wheel_forward @12.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @9 @0.3 @nowait", "wheel_left @2 @0.3 @nowait"},
                {"imuturn @92 @0.2", "wheel_back @64 @0.4 @yaw92", "check @imu @92", "imuturn @92 @0.2",}, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                {"sleep @10", "wheel_right @27 @0.4", "wheel_back @27 @0.4 @yaw92", },// driving to the apriltag
                {"wheel_left @22.5 @0.4"},
                {parkingCenter ? "wheel_right @27 @0.6" :"wheel_left @27 @0.6", "wheel_back @10 @0.5", "slider @min"},
                {"wheel_left @4 @0.2 @nowait"}, //mark7
            }, // center
            {
                {"wheel_forward @11 @0.3 @nowait"},
                {"nextstep @markRedFarRightIndex1"},
                //{"exit", "sleep @200", "wheel_turn_right @180 @0.3 @nowait", "wheel_left @1.0 @0.3 @nowait", "wheel_forward @14.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @12 @0.3 @nowait",}
                {"arm @moveready", "sleep @200", "grip @close", },
                {"sleep @10", "balance", "wheel_forward @80 @0.4", }, // driving to the backdrop board, originally the distance is too long (88), separate to two actions
                // first wheel_right more to avoid big colision with ally robot, then wheel_left a little.
                {"wheel_turn_right @180 @0.4", "wheel_left @35.5 @0.3 @nowait", "wheel_back @15 @0.3 @nowait"},// driving to the apriltag
                {"wheel_left @27.5 @0.3"}, // go through the center
                {parkingCenter ? "wheel_right @31 @0.8 @nowait" :"wheel_left @22 @0.6 @nowait", "wheel_back @10 @0.5 @nowait"},
                {""}, //mark7
            }, // right
        };

        markRedFarRightIndex1 = new String[] {
            "wheel_left @8 @0.3 @nowait",
            "wheel_forward @15 @0.3 @nowait",
            "wheel_turn_right @90 @0.3 @nowait",
            "wheel_forward @2 @0.3 @nowait"
        };

        // for blue
        // (after drop the purple pixel, turn *right*)
        markBlueNear = new String[][][] {
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_left @13 @0.3 @nowait"},
                {"wheel_turn_right @90 @0.3 @nowait"},
                {"wheel_back @26 @0.8"}, //{"wheel_back @28 @0.8"}, // driving to the backdrop board
                {"wheel_left @12.5 @0.2 @nowait"},//{"wheel_left @8 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @34 @0.6" : "wheel_right @23 @0.6", "wheel_back @11 @0.5" }, //{parkingCenter ? "wheel_left @38 @0.6" : "wheel_right @21 @0.6", "wheel_back @10 @0.5" },
            }, // left
            {
                {"wheel_forward @23 @0.4 @nowait"},
                {"wheel_left @7 @0.2 @nowait"},
                {"wheel_turn_right @90 @0.3 @nowait"},
                {"wheel_back @33 @0.8"}, // driving to the backdrop board
                {"wheel_left @6 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @26 @0.6" : "wheel_right @30.5 @0.6", "wheel_back @11 @0.5" },
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"nextstep @markBlueRightIndex1"},
                {""},
                {"wheel_back @35 @0.8"},// driving to the backdrop board
                {"wheel_left @8 @0.2 @nowait"}, // driving to the apriltag
                {""},
                {parkingCenter ? "wheel_left @22 @0.6" : "wheel_right @37 @0.6", "wheel_back @11 @0.5" },
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
                 // the wheel_back and wheel_forward operation is to avoid run over the team prop (by the arm), which may impact the robot move
                //{"exit", "wheel_back @5 @0.3 @nowait", "wheel_turn_right @180 @0.3 @nowait", "wheel_left @6 @0.3 @nowait", "", "wheel_forward @7.5 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @9.5 @0.3 @nowait",},
                {"arm @moveready", "sleep @200", "grip @close", },
                {"sleep @10", "balance", "wheel_forward @80 @0.4",}, // driving to the backdrop board, originally the distance is too long, separate to two actions
                // split the wheel_left to two parts to avoid drive over the purple pixel may be dropped by ally robot
                {"wheel_turn_right @180 @0.4", "wheel_right @31 @0.3 @nowait", "wheel_back @15 @0.3 @nowait"}, // driving to the apriltag
                {"wheel_right @31 @0.3"}, // drive to center lane
                {parkingCenter ? "wheel_left @34 @0.8 @nowait" : "wheel_right @22 @0.6 @nowait", "wheel_back @10 @0.5 @nowait"},
                {""}, //mark7
            }, // left
            {
                {"wheel_forward @23 @0.3 @nowait"},
                {"wheel_right @6 @0.3 @nowait"},
                // split the following line wheel_forward to 2 operations to avoid run over the purple pixel just dropped
                {"sleep @10", "wheel_turn_right @90 @0.3 @nowait", "wheel_forward @8 @0.3 @nowait", "wheel_left @7 @0.3 @nowait", "balance", "wheel_forward @4 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "", "wheel_back @9 @0.3 @nowait", },
                {"imuturn @-92 @0.2", "wheel_back @60 @0.4 @yaw-92", "check @imu @-92", "imuturn @-92 @0.2"}, // mark3, driving to the backdrop board
                    // split the wheel_left to two operations to avoid run over the purple pixel in ally field
                {"sleep @10", "wheel_left @22 @0.4 @nowait", "wheel_back @30 @0.4 @nowait @yaw-92", "", "wheel_left @7 @0.3 @nowait"}, // mark4, driving to the apriltag
                {"wheel_right @28 @0.4 @nowait"}, // mark5, drive to tross
                {parkingCenter ? "wheel_left @26 @0.6" :"wheel_right @30 @0.6", "wheel_back @10 @0.5", "slider @min"}, // mark6, parking
                {"wheel_forward @2 @0.2 @nowait", "wheel_left @8 @0.2 @nowait", "wheel_back @2 @0.2 @nowait",}, //mark7, drop white pixel
            }, // center
            {
                {"wheel_forward @12 @0.3 @nowait"},
                {"wheel_right @12 @0.3 @nowait"},
                // following line the first wheel_forward and wheel_back is to push the team prop away, otherwise it may stuck in the wheels
                {"wheel_forward @22 @0.3 @nowait", "wheel_back @5 @0.3 @nowait", "wheel_turn_right @90 @0.3 @nowait", "balance", "wheel_forward @6 @0.3 @nowait", "nextstep @presetActionsGetOnePixelFromStack", "wheel_back @2.5 @0.3 @nowait",},
                {"imuturn @-92 @0.2", "wheel_back @70 @0.4 @yaw-92", "check @imu @-92", "imuturn @-92 @0.2"}, // driving to the backdrop board
                // wheel_left split to two parts, to avoid big collision with ally's robot which could park in the center
                {"sleep @10", "wheel_left @30 @0.4 @nowait", "wheel_back @28 @0.4 @nowait @yaw-92", "wheel_left @5 @0.3 @nowait"}, // mark4, driving to the apriltag
                {"wheel_right @28 @0.4 @nowait"}, // mark5, drive to tross
                {parkingCenter ? "wheel_left @18 @0.6 @nowait" :"wheel_right @36 @0.6 @nowait", "wheel_back @10 @0.5", "slider @min"}, // mark6, parking
                {"wheel_forward @2 @0.2 @nowait", "wheel_right @4 @0.2 @nowait", "wheel_back @2 @0.2 @nowait",}, //mark7, drop white pixel
            }, // right
        };

        presetActionsDropPurpleOnly = new String[]{
                "ai_getmarkposition",
                "grip @close",
                "door @close",
                "box @pickup",
                "sleep @200",
                "arm @vertical",
                "ai_getmarkposition",
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
        };

        String[] presetActionsDropPurpleOnlyBak = new String[]{
            "grip @close",
            "door @close",
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            "ai_getmarkposition",
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
            "grip @close",
            "balance @redfar @bluefar",
        };

        presetActionsRedNear = new String[]{
            "ai_getmarkposition",
            "grip @close",
            "door @close",
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            "ai_getmarkposition",
            "camera @close",
            //"log",// for test only!!!
            //"exit",// for test only!!!
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
            //testWheelBalance ? "sleep @8000" : "",
            //"balance",
            "arm @vertical",
            continueRunToBoard ? "thread @distanceslider @-0.1 @0.1 @3200" : "thread @distanceslider @-0.1 @0.2 @1500",
            "slider @row0 @1500",
            "slider @0.005",
            "box @drop",
            "sleep @1500",
            "door @open",
            "sleep @500",
            "slider @upbig",
            "slider @0.005",
            "sleep @500",
            "wheel_forward @3 @0.3 @nowait",
            "door @close",
            "box @pickup",
            "slider @min",
            (parkingCenter && getWhitePixels) ?  "arm @high4" : "arm @moveready",
            (parkingCenter && getWhitePixels) ?  "grip @open" : "grip @small",
            "nextstep @mark6",
            //"checktimepassedlessthan @15000", //passed time should be less than some milliseconds, then do the next step
            //"nextstep @presetActionsGetPixels",
            "log",
        };

        presetActionsRedFar = new String[]{
            "ai_getmarkposition",
            "grip @close",
            "door @close",
            "box @pickup",
            "sleep @200",
            "arm @vertical",
            //"ai_getmarkposition",
            "camera @close",
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
            "grip @close",
             //useIMU ? "imuturn @92 @0.2" : "balance @redfar @bluefar",
            //"exit", // test only
            "nextstep @mark3",
            "nextstep @mark4",
            "arm @vertical",
            continueRunToBoard ? "thread @distanceslider @-0.1 @0.1 @3000" : "thread @distanceslider @-0.1 @0.2 @1500",
            "slider @row1far @1000",
            "slider @0.005",
            "box @drop",
            "sleep @1200",
            "door @openone",
            "sleep @300",
            "slider @up",
            "slider @0.005",
            "door @close",
            "set @wheelslider @off", // stop the distanceslider thread
            //"sleep @300",
            "nextstep @mark7",
            "set @wheelslider @on", // enable the distanceslider thread
            "thread @distanceslider @-0.1 @0.1 @1000",
            "sleep @500",
            useTwoWayDoor ? "door @open" : "",
            useTwoWayDoor ? "sleep @500" : "",
            "wheel_forward @2 @0.3 @nowait",
            "box @pickup",
            "slider @min",
            "door @close",
            "arm @moveready",
            workingMode.contains("far") && parkingAtBackDrop ? "" : "nextstep @mark6",
            "log",
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
                "slider @row2",
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
                debugMode ? "slider @row3 @1500": "",
                debugMode ? "slider @0.005": "",
                debugMode ? "box @drop": "",
                debugMode ? "exit" : "",
                //"set @scriptstopable @yes",
                "checkdistanceanddrivetobackdrop @8",
                //"checkdistanceanddrivetobackdrop @9",
                "manual_wheel @disable",
                continueRunToBoard ? "thread @distanceslider @-0.1 @0.1 @3500" : "thread @distanceslider @-0.1 @0.2 @1500", //"distanceslider @0.1 @0.2 @1500",
                "manual_wheel @enable",
                "grip @close",
                "arm @vertical",
                "slider @row5 @2000",//"thread @slider @max @2000",
                "slider @0.005",
                //"sleep @1000",
                "box @drop",
                "sleep @1500",//"sleep @1200",
                useTwoWayDoor ? "door @openone" : "door @open",
                "sleep @300",
                "door @close",
                //"sleep @320",
                "door @open",
                "sleep @800",
                "manual_wheel @disable",
                "wheel_forward @3 @0.3 @nowait",
                "manual_wheel @enable",
                //"set @drivespeed @0.1",
                "door @close",
                "box @pickup",
                "slider @min @1300",
                "arm @moveready",
                //"set @scriptstopable @no",
        };

        presetActionsPad1Down = new String[]{
                debugMode ? "slider @row1 @1500": "",
                debugMode ? "slider @0.005": "",
                debugMode ? "box @drop": "",
                debugMode ? "exit" : "",
                //"set @scriptstopable @yes",
                "checkdistanceanddrivetobackdrop @8",
                "manual_wheel @disable",
                continueRunToBoard ? "thread @distanceslider @-0.1 @0.1 @2800" : "thread @distanceslider @-0.1 @0.2 @1500", //"distanceslider @0.1 @0.2 @1500",
                "manual_wheel @enable",
                "grip @close",
                "arm @vertical",
                "slider @row1 @1500", //"thread @slider @high2 @1500",
                "slider @0.005",
                //"sleep @500",
                "box @drop",
                "sleep @1500",//"sleep @1000",
                useTwoWayDoor ? "door @openone" : "door @open",
                "sleep @300",
                "door @close",
                //"sleep @120",
                //"box @drop",
                "slider @up",
                "slider @0.005",
                "door @open",
                "sleep @800", //sleep @500
                "manual_wheel @disable",
                "wheel_forward @3 @0.3 @nowait",
                "manual_wheel @enable",
                //"set @drivespeed @0.1",
                "door @close",
                "box @pickup",
                "slider @min",
                "arm @moveready",
                //"set @scriptstopable @no",
        };

        presetActionsPad1Left = new String[]{
                //debugMode ? "slider @row4 @1500": "",
                //debugMode ? "slider @0.005": "",
                //debugMode ? "box @drop": "",
                //debugMode ? "arm @high5": "",
                debugMode ? "grip @close" : "",
                debugMode ? "sleep @500" : "",
                debugMode ? "arm @up" : "",
                debugMode ? "sleep @1000" : "",
                debugMode ? "grip @small" : "",
                debugMode ? "exit" : "",
                //"set @scriptstopable @yes",
                "checkdistanceanddrivetobackdrop @8",
                "manual_wheel @disable",
                "thread @distanceslider @-0.1 @0.1 @1500",
                "manual_wheel @enable",
                "grip @close",
                "arm @vertical",
                "thread @slider @row2 @1000",
                "sleep @500",
                "box @drop",
                //"set @drivespeed @0.1",
                "sleep @500",//"sleep @1000",
                //"set @scriptstopable @no",
        };

        presetActionsPad1Right = new String[]{
                //debugMode ? "slider @row2 @1500": "",
                //debugMode ? "slider @0.005": "",
                //debugMode ? "box @drop": "",
                debugMode ? "grip @close" : "",
                debugMode ? "sleep @100" : "",
                debugMode ? "arm @high5" : "",
                debugMode ? "sleep @300" : "",
                debugMode ? "grip @large" : "",
                // debugMode ? "grip @pos @0.5": "",
                debugMode ? "exit" : "",
                //"set @scriptstopable @yes",
                "checkdistanceanddrivetobackdrop @8",
                "manual_wheel @disable",
                continueRunToBoard ? "thread @distanceslider @-0.1 @0.1 @3200" : "thread @distanceslider @-0.1 @0.2 @1500", //"distanceslider @0.1 @0.2 @1500",
                "manual_wheel @enable",
                "grip @close",
                "arm @vertical",
                "slider @row3 @1500",
                "slider @0.005",
                //"sleep @500",
                "box @drop",
                "sleep @1500",//"sleep @1000",
                useTwoWayDoor ? "door @openone" : "door @open",
                useTwoWayDoor ? "sleep @300" : "sleep @180",
                "door @close",
                //"sleep @320",
                //"door @open",
                //"sleep @500",
                useTwoWayDoor ? "" : "sleep @120",
                "slider @up",
                "slider @0.005",
                "door @open",
                "sleep @800",
                "manual_wheel @disable",
                "wheel_forward @3 @0.3 @nowait",
                "manual_wheel @enable",
                //"set @drivespeed @0.1",
                "door @close",
                "box @pickup",
                "slider @min @1200",
                "arm @moveready",
                //"set @scriptstopable @no",
        };

        presetActionsPad1X = new String[]{
                "grip @close",
                "arm @moveready"
                //"lifter @0.4"
        };
        presetActionsPad1Y = new String[]{
                //"set @lifterarmpower @off"
                "grip @close",
                "sleep @500",
                "manual_wheel @disable",
                "wheel_back @3 @0.3 @nowait",
                "manual_wheel @enable",
                "arm @up",
                "sleep @1000",
                "grip @small",
        };

        presetActionsPad1A = new String[]{
                //"set @lifterarmpower @on"
                "grip @close",
                "sleep @100",
                "arm @down",
                "sleep @300",
                "grip @large",
        };

        presetActionsPad1B = new String[]{
                debugMode ? "": "grip @close",
                debugMode ? "arm @high5": "arm @vertical"
                //"ai_getmarkposition",
                //"lifter @-0.4"
        };

        presetActionsPad1UpWithRightBumper = new String[]{
                debugMode ? "distanceslider @0.1 @0.15 @1000" : "",
                //"nextstep @mark0",
                //"nextstep @mark1",
                //"recordposition @stop",
        };
        presetActionsPad1DownWithRightBumper = new String[]{
                //"nextstep @presetActionsRedNear"
        };
        presetActionsPad1LeftWithRightBumper = new String[]{
                //"grip @close",
                //"door @close",
                //"box @pickup",
                //"sleep @200",
                //"arm @vertical",
                //"ai_getmarkposition @5",
        };
        presetActionsPad1RightWithRightBumper = new String[]{
                //"arm @down",
                //"sleep @600",
                //"grip @small",
                //"sleep @200",
                //"arm @moveready",
                //"sleep @300",
                //"grip @close",
        };
        presetActionsPad1UpWithLeftBumper = new String[]{
                debugMode ? "" : "exit",
                "nextstep @mark5",
                "sleep @100",
                "nextstep @mark3",
                "nextstep @mark4",
                "balance @redfar @bluefar",
                "distanceslider @0.1 @0.2 @1500",
                "arm @vertical",
                "slider @row2",
                "sleep @300",
                "box @droplow",
        };
        presetActionsPad1DownWithLeftBumper = new String[]{
                debugMode ? "" : "exit",
                "wheel_forward @3 @0.2",
                "box @pickup",
                "slider @min",
                "arm @moveready",
                "nextstep @mark6",
        };
        presetActionsPad1LeftWithLeftBumper = new String[]{
                debugMode ? "" : "exit",
                "nextstep @mark2",
        };
        presetActionsPad1RightWithLeftBumper = new String[]{
                debugMode ? "" : "exit",
                "door @open",
                "sleep @1000",
                "door @close",
        };
        presetActionsPad1XWithLeftBumper = new String[]{
                //"ai_getmarkposition"
                //"door @open",
                //dropWhitePixelLater ? "sleep @180": "sleep @500",
                //dropWhitePixelLater ? "door @close": "",
                //dropWhitePixelLater ? "sleep @320": "",
                //dropWhitePixelLater ? "door @open": "",
                //dropWhitePixelLater ? "sleep @500": "",
                //dropWhitePixelLater ? "door @close": "",
                //"lifter @0.1",
                //"sleep @2000",
                "lifter @0.0",
        };
        presetActionsPad1YWithLeftBumper = new String[]{
                //"lifter @-0.4",
                //"lifterhelper @1.0",
                //"sleep @11000",
                //"lifter @0.0",
                //"lifterhelper @0.0"
                debugMode ? "set @armpower @off" : "",
                debugMode ? "set @lifterarmpower @off" : "",
                debugMode ? "lifter @0.1" : "",
        };
        presetActionsPad1AWithLeftBumper = new String[]{
                //useFastLifter ? "": "lifterhelper @0.0",
                debugMode ? "set @armpower @off" : "",
                debugMode ? "set @lifterarmpower @off" : "",
                debugMode ? "lifter @-0.1" : "",
        };
        presetActionsPad1BWithLeftBumper = new String[]{
                //"door @open",
                //"sleep @150",
                //"door @close",
                //useFastLifter ? "": "lifterhelper @1.0 @5000"
                //"lifter @0.0",
                debugMode ? "set @lifterarmpower @off" : ""
                //"ai_getmarkposition",
        };
        presetActionsPad1XWithRightBumper = new String[]{
                //"wheel_turn_left @90 @0.3"
                //"lifter @0.8"
                "lifterarm @-0.01"
        };

        presetActionsPad1YWithRightBumper = new String[]{
                hangFarSide ? "" : "exit",
                debugMode ? "exit" : "",
                //"check @hangready @off",
                "thread @lifterarm @up",
                "sleep @300",
                "set @armpower @off",
                //"set @lifterarmpower @off",
                //useFastLifter ? "": "lifter @-0.4",
                //useFastLifter ? "": "lifterhelper @1.0",
                //useFastLifter ? "": "sleep @1000",
                //useFastLifter ? "": "lifter @0.0",
                //useFastLifter ? "": "lifterhelper @0.0",
                "lifter @0.5",
                "sleep @1000",//"sleep @950",
                "lifter @0.0",
                "set @hangready @on",
        };
        presetActionsPad1AWithRightBumper = new String[]{
                debugMode ? "exit" : "",
                "check @hangready @on",
                "set @armpower @off",
                "set @lifterarmpower @off",
                //"wheel_turn_right @90 @0.4"
                //useFastLifter ? "" : "lifter @0.8",
                //useFastLifter ? "" : "sleep @3000",
                "lifter @-1.0",
                "sleep @1400",
                "lifter @-0.2",
                //"lifter @-0.1",
        };
        presetActionsPad1BWithRightBumper = new String[]{
                //"wheel_turn_right @180 @0.4"
                //"lifter @-0.8"
                useFastLifter ? "": "lifter @0.8",
                useFastLifter ? "": "sleep @1000",
                useFastLifter ? "": "lifter @0.2",
                "lifterarm @0.01"
        };

        presetActionsWheelTurnLeft = new String[]{
                "wheel_turn_left @90 @0.2"
        };

        presetActionsWheelTurnRight = new String[]{
                "wheel_turn_right @90 @0.2"
        };

        presetActionsPad1Back = new String[]{
                "drone @open",
                "set @drivespeed @1.0",
                "lifterarm @down",
                hangFarSide ? "exit" : "",
                debugMode ? "exit" : "",
                "sleep @1000",
                "thread @lifterarm @up",
                "sleep @300",
                "set @armpower @off",
                "lifter @0.5",
                "sleep @1000",//"sleep @950",
                "lifter @0.0"
        };

        presetActionsPad1Start = new String[] {
        };

        presetActionsPad1LeftTrigger = new String[]{
        };
        presetActionsPad1RightTrigger = new String[]{
        };

        presetActionsPad1LeftBumper = new String[]{
        };
        presetActionsPad1RightBumper = new String[]{
                "set @drivespeed @1.0",
        };

        presetActionsPad2Up = new String[]{
                debugMode ? "manual_wheel @disable" : "",
                debugMode ? "wheel_forward @24 @0.2 @yaw0" : "",
                debugMode ? "manual_wheel @enable" : "",
                debugMode ? "exit" : "",
                "arm @vertical",
                "slider @up",
                "slider @0.003",
                "box @drop"
        };
        presetActionsPad2Down = new String[]{
                debugMode ? "manual_wheel @disable" : "",
                debugMode ? "wheel_back @48 @0.3 @yaw0" : "",
                debugMode ? "manual_wheel @enable" : "",
                debugMode ? "exit" : "",
                //"box @pickup",
                "slider @down",
                "slider @0.003",
        };
        presetActionsPad2Left = new String[]{
                debugMode ? "manual_wheel @disable" : "",
                debugMode ? "wheel_back @60 @0.4 @yaw92" : "",
                debugMode ? "manual_wheel @enable" : "",
                debugMode ? "exit" : "",
                "box @pickup"
        };
        presetActionsPad2Right = new String[]{
                debugMode ? "manual_wheel @disable" : "",
                debugMode ? "imuturn @91 @0.2" : "",
                debugMode ? "wheel_back @60 @0.4 @yaw91" : "",
                debugMode ? "manual_wheel @enable" : "",
                debugMode ? "exit" : "",
                //debugMode ? "slider @firstrowfar" : "",
                "box @drop"
        };

        presetActionsPad2UpWithLeftBumper = new String[]{
                "slider @0.2"
        };
        presetActionsPad2DownWithLeftBumper = new String[]{
                "slider @-0.2"
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
                "slider @0.005",
                "box @drop",
                "slider @max",
                "slider @0.003",
        };
        presetActionsPad2DownWithRightBumper = new String[]{
                "box @pickup",
                "grip @close",
                "door @close",
                "sleep @200",
                //"slider @down",
                "slider @min",
                "arm @moveready"
        };

        presetActionsPad2LeftWithRightBumper = new String[]{
                "exit",
                "slider @row3"
        };
        presetActionsPad2RightWithRightBumper = new String[]{
                "exit",
                "slider @row5"
        };

        presetActionsPad2X = new String[]{
                debugMode ? "imuturn @-86 @0.2" : "grip @large"
        };
        String[] presetActionsPad2YBak = new String[]{
                debugMode ? "imuturn @-88 @0.2" : "grip @close",
                debugMode ? "exit" : "",
                "grip @close",
                "sleep @500",
                "manual_wheel @disable",
                "wheel_back @3 @0.3 @nowait",
                "manual_wheel @enable",
                "arm @up",
                "sleep @1000",
                "grip @small",
                //"sleep @100",
                //"arm @default",
                //"sleep @300",
                //"box @pixelslide",
                //"sleep @200",
                //"box @pickup"
                //"set @drivespeed @1.0",
        };
        presetActionsPad2Y = new String[]{
                debugMode ? "imuturn @-88 @0.2" : "grip @close",
                debugMode ? "exit" : "",
                "grip @close",
                "sleep @500",
                "manual_wheel @disable",
                "wheel_back @3 @0.3 @nowait",
                "manual_wheel @enable",
                "arm @up",
                "sleep @1000",
                "grip @small",
                //"sleep @100",
                //"arm @default",
                //"sleep @300",
                //"box @pixelslide",
                //"sleep @200",
                //"box @pickup"
                //"set @drivespeed @1.0",
        };
        presetActionsPad2A = new String[]{
                debugMode ? "imuturn @92 @0.2" : "grip @close",
                debugMode ? "exit" : "",
                "grip @close",
                "sleep @100",
                "arm @down",
                "sleep @300",
                "grip @large",
                //"set @drivespeed @1.0",
        };
        presetActionsPad2B = new String[]{
                debugMode ? "imuturn @90 @0.2" : "grip @close"
        };

        presetActionsPad2XWithRightBumper = new String[]{
                "door @0.01",
                "exit",
                "manual_wheel @disable",
                "wheel_left @4 @0.2 @nowait",
                "wheel_back @3 @0.2 @nowait",
                "grip @close",
                "arm @moveready",
                "manual_wheel @enable",
        };

        presetActionsPad2YWithRightBumper = new String[]{
                "arm @firstrow",
                "sleep @300",
                "grip @verylarge",
        };
        presetActionsPad2AWithRightBumper = new String[]{
                //"door @close"
                "arm @secondrow",
                "sleep @300",
                "grip @verylarge",
        };
        presetActionsPad2BWithRightBumper = new String[]{
                "door @-0.01",
                "exit",
                "manual_wheel @disable",
                "wheel_right @4 @0.2 @nowait",
                "wheel_back @3 @0.2 @nowait",
                "grip @close",
                "arm @moveready",
                "manual_wheel @enable",
        };
        presetActionsPad2XWithLeftBumper = new String[]{
                "grip @0.01"
        };
        presetActionsPad2YWithLeftBumper = new String[]{
                "arm @-0.01"
        };
        presetActionsPad2AWithLeftBumper = new String[]{
                "arm @0.01"
        };
        presetActionsPad2BWithLeftBumper = new String[]{
                "grip @-0.01"
        };
        presetActionsPad2XWithLeftStickX = new String[]{
                "door @openone"
        };
        presetActionsPad2YWithLeftStickX = new String[]{
                "grip @open",
        };
        presetActionsPad2AWithLeftStickX = new String[]{
                "door @open",
        };
        presetActionsPad2BWithLeftStickX = new String[]{
                "door @close"
        };
        presetActionsPad2LeftStick = new String[]{
                "arm @moveready",
        };
        presetActionsPad2RightStick = new String[]{
                "arm @vertical",
        };
        presetActionsPad2LeftTrigger = new String[]{
                useTwoWayDoor ? "door @openone" : "door @open",
                useTwoWayDoor ? "sleep @300" : "sleep @800",
                "door @close",
        };
        presetActionsPad2RightTrigger = new String[]{
                //"door @open",
                //useTwoWayDoor ? "sleep @500" : "sleep @150",
                //"door @close",
                useTwoWayDoor ? "door @openone" : "door @open",
                useTwoWayDoor ? "sleep @300" : "sleep @800",
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
