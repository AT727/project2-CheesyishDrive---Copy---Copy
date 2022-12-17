package frc.robot;

public final class Constants {
    //1678 citrus circutis
    // public static final double kJoystickThreshold = 0.2;
    // public static final int kMainTurnJoystickPort = 0;
    // public static final int kMainThrottleJoystickPort = 1;
    // public static final int kButtonGamepadPort = 2;
    // public static final double kDriveWheelTrackWidthInches = 31.170; //change this
    // public static final double kTrackScrubFactor = 1.0; // Tune me!

    //can spark and drive
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveId1 = 2;
    public static final int kRightDriveSlaveId2 = 3;
    public static final int kLeftDriveMasterId = 4;
    public static final int kLeftDriveSlaveId1 = 5;
    public static final int kLeftDriveSlaveId2 = 6;
    public static final double kDriveVoltageRampRate = 0.0;

    //team 27 constants. used for limiting motors so they don't go crazy fast, SHOULD TUNE
    public static int MAX_PEAK_CURRENT = 37;
    public static int MAX_CONTINUOUS_CURRENT = 18;


    //control board
    public static final boolean kUseDriveGamepad = false; 
    public static final double kJoystickThreshold = 0.2;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainTurnJoystickPort = 1; 
    public static final int kMainThrottleJoystickPort = 0; 
    public static final int kDriveGamepadPort = 0;
   

    //kinematics
    public static final double kTrackScrubFactor = 1.0469745223;
    public static final double kDriveWheelTrackWidthInches = 25.54; //prone to change
}
