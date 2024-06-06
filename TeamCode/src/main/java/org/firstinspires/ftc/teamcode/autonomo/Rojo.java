package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class Rojo  extends LinearOpMode {

    // Declare OpMode members. ElapsedTime();
    private DcMotor RightF;
    private DcMotor RightB;
    private DcMotor LeftF;
    private DcMotor LeftB;
    private DcMotor Intake;
    private DcMotor ElevR;
    private DcMotor ElevL;
    private Servo Leaver;

    Conversion conversion = new Conversion();

    //Set the target for all the motors
    public void setMotorsTarget(int rf, int rb, int lf, int lb){
        RightF.setTargetPosition(rf);
        RightB.setTargetPosition(rb);
        LeftF.setTargetPosition(lf);
        LeftB.setTargetPosition(lb);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // rf = Right Forward rb = Right Backward lf = Left Forward lb = Left Backward

        int rf = 0;
        int rb = 0;
        int lf = 0;
        int lb = 0;
        int count = 1;

        int velocity = 1500;

        RightF  = hardwareMap.get(DcMotor.class, "RightF");
        RightB = hardwareMap.get(DcMotor.class, "RightB");

        LeftF  = hardwareMap.get(DcMotor.class, "LeftF");
        LeftB = hardwareMap.get(DcMotor.class, "LeftB");

        ElevL  = hardwareMap.get(DcMotor.class, "ElevL");
        ElevR = hardwareMap.get(DcMotor.class, "ElevR");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Leaver = hardwareMap.get(Servo.class, "Leaver");

        LeftB.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftF.setDirection(DcMotorSimple.Direction.REVERSE);

        RightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Init the target position to 0 in all the motors
        setMotorsTarget(rf,rb,lf,lb);

        RightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx)RightF).setVelocity(velocity);
        ((DcMotorEx)RightB).setVelocity(velocity);
        ((DcMotorEx)LeftF).setVelocity(velocity);
        ((DcMotorEx)LeftB).setVelocity(velocity);

        telemetry.addData("Valores inciales: ", "%7d :%7d",
                RightF.getCurrentPosition(),
                LeftF.getCurrentPosition(),
                RightB.getCurrentPosition(),
                LeftB.getCurrentPosition()
        );

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =   RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection  =   RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            imu.getRobotYawPitchRollAngles();

            if(count == 1){
                count = 0;

                RightF.setDirection(DcMotorSimple.Direction.REVERSE);
                RightB.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftF.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftB.setDirection(DcMotorSimple.Direction.FORWARD);

                RightF.setTargetPosition(conversion.INCH_TO_TICKS(2));
                RightB.setTargetPosition(conversion.INCH_TO_TICKS(2));
                LeftF.setTargetPosition(conversion.INCH_TO_TICKS(2));
                LeftB.setTargetPosition(conversion.INCH_TO_TICKS(2));
                while(RightF.isBusy() || RightB.isBusy() || LeftB.isBusy() || LeftF.isBusy()){
                    telemetry.addData("Status: ", " Arriving");
                }

                RightF.setDirection(DcMotorSimple.Direction.FORWARD);
                RightB.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftF.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftB.setDirection(DcMotorSimple.Direction.REVERSE);
                RightF.setTargetPosition(conversion.INCH_TO_TICKS(35));
                RightB.setTargetPosition(conversion.INCH_TO_TICKS(35));
                LeftF.setTargetPosition(conversion.INCH_TO_TICKS(35));
                LeftB.setTargetPosition(conversion.INCH_TO_TICKS(35));

                while(RightF.isBusy() || RightB.isBusy() || LeftB.isBusy() || LeftF.isBusy()){
                    telemetry.addData("Status: ", " Arriving");
                }

                telemetry.addData("Valores inciales: ", "%7d :%7d",
                        RightF.getCurrentPosition(),
                        LeftF.getCurrentPosition(),
                        RightB.getCurrentPosition(),
                        LeftB.getCurrentPosition()
                );
            };
            telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Roll: ", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Pitch: ", orientation.getPitch(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}
