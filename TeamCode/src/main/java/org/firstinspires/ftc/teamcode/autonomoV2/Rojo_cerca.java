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
public class Rojo_cerca extends LinearOpMode{
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

            //Left and right
            else if(prop_position == 1 || prop_position == 2){

                //Turn to the left
                RightF.setTargetPosition(rightF - 850);
                RightB.setTargetPosition(rightB - 850);
                LeftF.setTargetPosition(leftF + 850);
                LeftB.setTargetPosition(leftB + 850);
                waitForArrive();
                encoderUpdate();
                while(orientation.getYaw(AngleUnit.DEGREES) > -86 || orientation.getYaw(AngleUnit.DEGREES) < -88) {
                    orientation = imu.getRobotYawPitchRollAngles();
                    RightF.setTargetPosition(rightF - 20);
                    RightB.setTargetPosition(rightB - 20);
                    LeftF.setTargetPosition(leftF + 20);
                    LeftB.setTargetPosition(leftB + 20);
                    telemetry.addData("Status: ", "Arriving");
                    telemetry.addData("Yaw (Z): ", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                    encoderUpdate();
                }

                //Left ---------------------
                if(prop_position == 1) {
                    //Moves to the spike mark
                    RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(2));
                    RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(2));
                    LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(2));
                    LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(2));
                    waitForArrive();
                    encoderUpdate();

                    //Left the pixel
                    Holder1.setPosition(0.6);
                    Wrist.setPosition(0.65);

                    //Moves backward to the center
                    RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(25));
                    RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(25));
                    LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(25));
                    LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(25));
                    waitForArrive();
                    encoderUpdate();
                }

                //Right --------------------
                if(prop_position == 2){
                    RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(21));
                    RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(21));
                    LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(21));
                    LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(21));
                    waitForArrive();
                    encoderUpdate();

                    //Left the pixel
                    Holder1.setPosition(0.6);
                    Wrist.setPosition(0.65);
                    waitForArrive();
                    encoderUpdate();

                    //Moves backward to the center
                    RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(4));
                    RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(4));
                    LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(4));
                    LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(4));
                    waitForArrive();
                    encoderUpdate();
                }
            }

            //----------------------------------

            //Moves to the backdrop
            if(prop_position == 0){
                //Track to the backdrop
                RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(35));
                RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(35));
                LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(35));
                LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(35));
                waitForArrive();
                encoderUpdate();

                RightF.setTargetPosition(rightF - 850);
                RightB.setTargetPosition(rightB - 850);
                LeftF.setTargetPosition(leftF + 850);
                LeftB.setTargetPosition(leftB + 850);
                while(orientation.getYaw(AngleUnit.DEGREES) > -86 || orientation.getYaw(AngleUnit.DEGREES) < -88) {
                    orientation = imu.getRobotYawPitchRollAngles();
                    RightF.setTargetPosition(rightF - 40);
                    RightB.setTargetPosition(rightB - 40);
                    LeftF.setTargetPosition(leftF + 40);
                    LeftB.setTargetPosition(leftB + 40);
                    telemetry.addData("Status: ", "Arriving");
                    telemetry.addData("Yaw (Z): ", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                    encoderUpdate();
                }

                waitForArrive();
                encoderUpdate();

            }

            //Gets close to the backdrop
            RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(17));
            RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(17));
            LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(17));
            LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(17));
            waitForArrive();
            encoderUpdate();

            //Alligns with the backdrop
            while(orientation.getYaw(AngleUnit.DEGREES) > -86 || orientation.getYaw(AngleUnit.DEGREES) < -88) {

                orientation = imu.getRobotYawPitchRollAngles();
                RightF.setTargetPosition(rightF - 40);
                RightB.setTargetPosition(rightB - 40);
                LeftF.setTargetPosition(leftF + 40);
                LeftB.setTargetPosition(leftB + 40);
                telemetry.addData("Status: ", "Arriving");
                telemetry.addData("Yaw (Z): ", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                encoderUpdate();
            }

            //Moves lateral to left
            if(prop_position == 1){
                RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(8));
                RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(8));
                LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(8));
                LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(8));
                waitForArrive();
                encoderUpdate();
            }

            //Moves lateral to right
            if(prop_position == 2){
                RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(9));
                RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(9));
                LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(9));
                LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(9));
                waitForArrive();
                encoderUpdate();
            }

            //Left the yellow pixel -------------

            while(orientation.getYaw(AngleUnit.DEGREES) > -88 || orientation.getYaw(AngleUnit.DEGREES) < -90) {
                orientation = imu.getRobotYawPitchRollAngles();
                RightF.setTargetPosition(rightF - 40);
                RightB.setTargetPosition(rightB - 40);
                LeftF.setTargetPosition(leftF + 40);
                LeftB.setTargetPosition(leftB + 40);
                telemetry.addData("Status: ", "Arriving");
                telemetry.addData("Yaw (Z): ", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                encoderUpdate();
            }

            //Levantar brazo
            Holder1.setPosition(0.60);
            Arm.setTargetPosition(1400);
            while (Arm.isBusy()) {
                telemetry.addData("Status: ", "Arriving");
            }

            sleep(200);
            RightF.setTargetPosition(rightF + cv.INCH_TO_TICKS(10));
            RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(10));
            LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(10));
            LeftB.setTargetPosition(leftB + cv.INCH_TO_TICKS(10));
            waitForArrive();
            encoderUpdate();

            Arm.setTargetPosition(32);
            while (Arm.isBusy()) {
                telemetry.addData("Status: ", "Arriving");
            }
            //-------------------------------------------

            //Park
            int distance = 0;

            if(prop_position == 0){
                distance = 60;
            }
            else if(prop_position == 1){
                distance = 45;
            }
            else if(prop_position == 2){
                distance = 30;
            }

            RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(distance));
            RightB.setTargetPosition(rightB + cv.INCH_TO_TICKS(distance));
            LeftF.setTargetPosition(leftF + cv.INCH_TO_TICKS(distance));
            LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(distance));
            waitForArrive();
            encoderUpdate();

            RightF.setTargetPosition(rightF - cv.INCH_TO_TICKS(20));
            RightB.setTargetPosition(rightB - cv.INCH_TO_TICKS(20));
            LeftF.setTargetPosition(leftF - cv.INCH_TO_TICKS(20));
            LeftB.setTargetPosition(leftB - cv.INCH_TO_TICKS(20));
            waitForArrive();
            encoderUpdate();
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