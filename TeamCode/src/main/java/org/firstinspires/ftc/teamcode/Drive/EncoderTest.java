package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class EncoderTest extends LinearOpMode {

    DcMotor FrontR;
    DcMotor FrontL;
    DcMotor BackR;
    DcMotor BackL;
    IMU imu;

    @Override
    public void runOpMode(){

        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackR = hardwareMap.get(DcMotor.class, "BackL");
        BackL = hardwareMap.get(DcMotor.class, "BackR");

        setUpEncoder(FrontR, 1000, DcMotorSimple.Direction.REVERSE);
        setUpEncoder(FrontL, 1000, DcMotorSimple.Direction.FORWARD);
        setUpEncoder(BackR, 1000, DcMotorSimple.Direction.FORWARD);
        setUpEncoder(BackL, 1000, DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                FrontR.setTargetPosition(1200);
                FrontL.setTargetPosition(1200);
                BackR.setTargetPosition(1200);
                BackL.setTargetPosition(1200);
            }

            else if(gamepad1.dpad_up){
                FrontR.setTargetPosition(0);
                FrontL.setTargetPosition(0);
                BackR.setTargetPosition(0);
                BackL.setTargetPosition(0);
            }
            telemetry.addData("Target RF: ", FrontR.getTargetPosition());
            telemetry.addData("Current RF: ", FrontR.getCurrentPosition());
            telemetry.addData("Target LF: ", FrontL.getTargetPosition());
            telemetry.addData("Current LF: ", FrontL.getCurrentPosition());
            telemetry.addData("Target RB: ", BackR.getTargetPosition());
            telemetry.addData("Current RB: ", BackR.getCurrentPosition());
            telemetry.addData("Target LB: ", BackL.getTargetPosition());
            telemetry.addData("Current LB: ", BackL.getCurrentPosition());

            telemetry.addData("Status: ", "Running");
            telemetry.update();
        }
    }

    public void setUpEncoder(DcMotor motor, int velocity, DcMotorSimple.Direction direction){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motor).setVelocity(velocity);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(direction);
    }
}
