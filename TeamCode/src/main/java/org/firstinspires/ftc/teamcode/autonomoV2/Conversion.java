package org.firstinspires.ftc.teamcode.autonomoV2;

public class Conversion {
    static final double     COUNTS_PER_MOTOR_REV    = 537;
//  static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.8;
    static final double     RADIAN_PER_TICK = (1 / COUNTS_PER_MOTOR_REV) * 6.2832;
    static final double     DISPLACEMENT_PER_TICK = RADIAN_PER_TICK * (WHEEL_DIAMETER_INCHES / 2);
//  static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public int INCH_TO_TICKS(double inches)
    {
        return (int)(inches / DISPLACEMENT_PER_TICK);
    }
}

