package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleOpOmni extends LinearOpMode {

    DcMotor FrontL;
    DcMotor FrontR;
    DcMotor BackL;
    DcMotor BackR;

    @Override
    public void runOpMode() throws InterruptedException{
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        double slower = 1.0;

        double  frontR;
        double  frontL;
        double  backR;
        double  backL;

        waitForStart();

        while(opModeIsActive()){
            frontL = -gamepad1.left_stick_y -gamepad1.left_stick_x -gamepad1.right_stick_x;
            frontR = +gamepad1.left_stick_y -gamepad1.left_stick_x -gamepad1.right_stick_x;
            backR = +gamepad1.left_stick_y +gamepad1.left_stick_x -gamepad1.right_stick_x;
            backL = -gamepad1.left_stick_y +gamepad1.left_stick_x -gamepad1.right_stick_x;

            FrontL.setPower(frontL);
            FrontR.setPower(frontR);
            BackR.setPower(backR);
            BackL.setPower(backL);
        }

    }
}
