package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderPID extends LinearOpMode {

    DcMotorEx motor;
    PIDController control = new PIDController(0.04,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "FrontR");

        // use braking to slow the motor down faster

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        int targetPosition = 1000;

        while (opModeIsActive()) {
            // update pid controller
            double command = control.update(targetPosition,
                    motor.getCurrentPosition());
            // assign motor the PID output
            motor.setPower(command);
            telemetry.addData("Error: ", control.error);
            telemetry.addData("Target: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}

