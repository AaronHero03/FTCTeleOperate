package org.firstinspires.ftc.teamcode.autonomoV2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Azul_largo extends LinearOpMode{
    int rightF = 0;
    int rightB= 0;
    int leftF= 0;
    int leftB= 0;
    //0 = Center, 1 = Right, 2 = Left
    int prop_position = 0;

    IMU imu;
    DcMotor RightF;
    DcMotor RightB;
    DcMotor LeftB;
    DcMotor LeftF;
    DcMotor Arm;
    Servo Wrist;
    Servo Holder;
    Servo Holder1;

    Conversion cv = new Conversion();

    @Override
    public void runOpMode() throws InterruptedException {
        RightF = hardwareMap.get(DcMotor.class, "RightF");
        RightB = hardwareMap.get(DcMotor.class, "RightB");
        LeftF = hardwareMap.get(DcMotor.class, "LeftF");
        LeftB = hardwareMap.get(DcMotor.class, "LeftB");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        imu = hardwareMap.get(IMU.class, "imu");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Holder = hardwareMap.get(Servo.class, "Holder");
        Holder1 = hardwareMap.get(Servo.class, "Holder1");
        Wrist.setDirection(Servo.Direction.REVERSE);
        Wrist.setPosition(0.205);
        Holder.setPosition(0.66);
        Holder1.setPosition(0.45);
        initEncoder(RightF, 1200, true);
        initEncoder(RightB, 1200, true);
        initEncoder(LeftF, 1200, false);
        initEncoder(LeftB, 1200, false);
        initEncoder(Arm, 1000, false);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            //Moves to the mat border
            RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(28));
            RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(28));
            LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(28));
            LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(28));
            waitForArrive();
            encoderUpdate();

            //Moves forward and left the pixel

            //Left the purple pixel ------------

            //Center
            if(prop_position == 0){
                RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(3));
                RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(3));
                LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(3));
                LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(3));
                waitForArrive();
                encoderUpdate();
                //Moves to the spike mark
                Holder1.setPosition(0.6);
                Wrist.setPosition(0.65);
            }

            //Left --------------------
            if(prop_position == 1) {
                //Moves to the spike mark
                RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(5));
                RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(5));
                LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(5));
                LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(5));
                waitForArrive();
                encoderUpdate();

                //Left the pixel
                Holder1.setPosition(0.6);
                Wrist.setPosition(0.65);

                //Moves to the spike mark
                RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(5));
                RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(5));
                LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(5));
                LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(5));
                waitForArrive();
                encoderUpdate();
            }

            //Right ---------------------
            if(prop_position == 2) {
                //Moves to the spike mark
                RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(5));
                RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(5));
                LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(5));
                LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(5));
                waitForArrive();
                encoderUpdate();

                //Left the pixel
                Holder1.setPosition(0.6);
                Wrist.setPosition(0.65);

                //Moves to the spike mark
                RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(5));
                RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(5));
                LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(5));
                LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(5));
                waitForArrive();
                encoderUpdate();
            }
        }
    }

    //Waits until the motors had arrived
    //It's necessary each time a motor target is set
    private void waitForArrive(){
        while (RightF.isBusy() || RightB.isBusy() || LeftF.isBusy() || LeftB.isBusy()) {
            telemetry.addData("Status: ", "Arriving");
            telemetry.addData("RightF: ", RightF.getCurrentPosition());
            telemetry.addData("RightB: ", RightB.getCurrentPosition());
            telemetry.addData("LeftF: ", LeftF.getCurrentPosition());
            telemetry.addData("LeftB: ", LeftB.getCurrentPosition());
            telemetry.update();
        }
    }

    //Sets the encoder mode for use with targets
    private void initEncoder(DcMotor motorI, int velocity, boolean direction) {
        motorI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorI.setTargetPosition(0);
        motorI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorI).setVelocity(velocity);
        motorI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (direction) {
            motorI.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motorI.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    //Update the current value of the motors
    //Don't forget to use it each time that a motor target is changed
    private void encoderUpdate() {
        rightF = RightF.getCurrentPosition();
        rightB = RightB.getCurrentPosition();
        leftF = LeftF.getCurrentPosition();
        leftB = LeftB.getCurrentPosition();
    }
}