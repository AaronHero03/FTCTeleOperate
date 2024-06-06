package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class Prueba_chasis extends LinearOpMode {

    DcMotor RightF;
    DcMotor RightB;
    DcMotor LeftB;
    DcMotor LeftF;

    @Override
    public void runOpMode() throws InterruptedException{

        RightF = hardwareMap.get(DcMotor.class, "RightF");
        RightB = hardwareMap.get(DcMotor.class, "RightB");
        LeftF = hardwareMap.get(DcMotor.class, "LeftF");
        LeftB = hardwareMap.get(DcMotor.class, "LeftB");
        usingEncoder(RightF);
        usingEncoder(RightB);
        usingEncoder(LeftF);
        usingEncoder(LeftB);

        double x = 0;
        double y = 0;
        double rx = 0;

        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        LeftF.setPower(frontLeftPower * 0.85);
        LeftB.setPower(backLeftPower * 0.85);
        RightF.setPower(frontRightPower * 0.85);
        RightB.setPower(backRightPower * 0.85);

        telemetry.addData("RightF: ", RightF.getCurrentPosition());
        telemetry.addData("RightL: ", RightB.getCurrentPosition());
        telemetry.addData("LeftF: ", LeftF.getCurrentPosition());
        telemetry.addData("LeftB: ", LeftB.getCurrentPosition());

        telemetry.update();
    }
    private void usingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
