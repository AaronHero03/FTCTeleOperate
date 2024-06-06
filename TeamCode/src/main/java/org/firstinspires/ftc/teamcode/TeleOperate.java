package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOperate extends LinearOpMode {

    DcMotor RightF;
    DcMotor RightB;
    DcMotor LeftB;
    DcMotor LeftF;
    DcMotor Intake;
    DcMotor Misumis;

    Servo Izquierda;
    Servo Derecha;
    Servo Giro;
    Servo Torque;
    DcMotor ElevR;
    DcMotor ElevL;
    CRServo IntakeS;

    @Override

    public void runOpMode() throws InterruptedException {

        double mismuis = 0;

        RightF = hardwareMap.get(DcMotor.class, "RightF");
        RightB = hardwareMap.get(DcMotor.class, "RightB");
        LeftF = hardwareMap.get(DcMotor.class, "LeftF");
        LeftB = hardwareMap.get(DcMotor.class, "LeftB");

        Izquierda = hardwareMap.get(Servo.class, "Izquierda");
        Derecha = hardwareMap.get(Servo.class, "Derecha");
        Torque = hardwareMap.get(Servo.class, "Torque");
        Giro = hardwareMap.get(Servo.class, "Giro");

        ElevR = hardwareMap.get(DcMotor.class, "ElevR");
        ElevL = hardwareMap.get(DcMotor.class, "ElevL");

        Intake = hardwareMap.get(DcMotor.class,"Intake");
        IntakeS = hardwareMap.get(CRServo.class, "IntakeS");
        Misumis = hardwareMap.get(DcMotor.class,"Misumis");
    

        /*
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Holder = hardwareMap.get(Servo.class, "Holder");
        Holder1 = hardwareMap.get(Servo.class, "Holder1");
        Leaver = hardwareMap.get(Servo.class, "Leaver");
        Hook = hardwareMap.get(Servo.class, "Hook");
        Hook1 = hardwareMap.get(Servo.class, "Hook1");*/

        usingEncoder(RightF);
        usingEncoder(RightB);
        usingEncoder(LeftF);
        usingEncoder(LeftB);
        usingEncoder(ElevR);
        usingEncoder(ElevL);

        initEncoder(Misumis, 2500, true);

        IntakeS.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftF.setDirection(DcMotor.Direction.REVERSE);
        LeftB.setDirection(DcMotor.Direction.REVERSE);
        ElevR.setDirection(DcMotor.Direction.FORWARD);
        ElevL.setDirection(DcMotor.Direction.FORWARD);
        Torque.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //GAMEPAD1

            //Meccanum
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
            LeftF.setPower(frontLeftPower * 1);
            LeftB.setPower(backLeftPower *1 );
            RightF.setPower(frontRightPower * 1);
            RightB.setPower(backRightPower * 1);

            if(gamepad1.a){
                Izquierda.setPosition(1);
            }
            if(gamepad1.b){
                Izquierda.setPosition(0);
            }
            if(gamepad1.x){
                Derecha.setPosition(0);
            }
            if(gamepad1.y){
                Derecha.setPosition(0.6);
            }

            if(gamepad2.a){
                Giro.setPosition(0);
            }
            if(gamepad2.b){
                Giro.setPosition(0.3);
            }

            if(gamepad2.x){
                Torque.setPosition(0.2);
            }
            if(gamepad2.y){
                Torque.setPosition(0.41);
            }

            // INTAKE
            if(gamepad1.left_trigger > 0){
                Intake.setPower(gamepad1.left_trigger);
                IntakeS.setPower(gamepad1.left_trigger);
            }

            if(gamepad1.left_trigger == 0){
                IntakeS.setPower(-gamepad1.right_trigger);
                Intake.setPower(-gamepad1.right_trigger);
            }

            // MISUMIS
            if(gamepad1.dpad_up){
                while(gamepad1.dpad_up){
                    mismuis += 10;
                    //Misumis.setTargetPosition(mismuis);
                    telemetry.addData("Misumis: ", (int)mismuis);
                    Misumis.setTargetPosition((int)mismuis);
                    telemetry.addData("Misumis Position: ", Misumis.getCurrentPosition());
                    telemetry.addData("Misumis Target Position: ", Misumis.getTargetPosition());
                    telemetry.update();
                }
            }
            if(gamepad1.dpad_down){
                while(gamepad1.dpad_down){
                    mismuis -= 10;
                    //Misumis.setTargetPosition(mismuis);
                    telemetry.addData("Misumis: ", (int)mismuis);
                    Misumis.setTargetPosition((int)mismuis);
                    telemetry.addData("Misumis Position: ", Misumis.getCurrentPosition());
                    telemetry.addData("Misumis Target Position: ", Misumis.getTargetPosition());
                    telemetry.update();
                }
            }


            // HANGING
            if (gamepad1.dpad_up) {
                ElevR.setTargetPosition(1550);
                ElevL.setTargetPosition(1550);
            }
            else if (gamepad1.dpad_down) {
                ElevR.setTargetPosition(0);
                ElevL.setTargetPosition(0);
            }

            /*
            //Leaver
             if (gamepad1.a) {
                Leaver.setPosition(0.5);
            } else if (gamepad1.y) {
                Leaver.setPosition(1);
            }

            //Drone Launcher
            if (gamepad1.b) {
                Drone.setPosition(0.5);
            } else if (gamepad1.x) {
                Drone.setPosition(0.75);
            }

            //GAMEPAD 2

            //Intake
            if (gamepad2.left_bumper) {
                Intake.setPower(0.8);
            } else if (gamepad2.right_bumper) {
                Intake.setPower(-0.8);
            }

            //Brazo
            if (gamepad2.options) {
                Arm.setTargetPosition(30);
                while (Arm.isBusy()) {
                    telemetry.addData("Status: ", "Arriving");
                }
            } else if (gamepad2.dpad_up) {
                //Levantar
                Wrist.setPosition(0.75);
                Arm.setTargetPosition(255);
                while (Arm.isBusy()) {
                    telemetry.addData("Status: ", "Arriving");
                }
            } else if (gamepad2.dpad_down) {
                Arm.setTargetPosition(0);
                while (Arm.isBusy()) {
                    telemetry.addData("Status: ", "Arriving");
                }
            }

            //Muñeca
            if (gamepad2.a) {
                Wrist.setPosition(0.6);
            } else if (gamepad2.y) {
                Wrist.setPosition(0);
            }

            //Garra izquierda
            if (gamepad2.b) {
                Holder.setPosition(0.35);
            } else if (gamepad2.x) {
                Holder.setPosition(0.3);
            }

            //Garra derecha
            if (gamepad2.dpad_left) {
                Holder1.setPosition(0.5);
            } else if (gamepad2.dpad_right) {
                Holder1.setPosition(0.55);
            }

            telemetry.addData("Arm: ", Arm.getCurrentPosition());
            telemetry.addData("Wrist: ", Wrist.getPosition());
            telemetry.addData("ElevL: ", ElevL.getCurrentPosition());
            telemetry.addData("ElevR: ", ElevR.getCurrentPosition());
            */
            telemetry.update();
        }
    }
    private void initEncoder(DcMotor motorI, int velocity, boolean direction){
        motorI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorI.setTargetPosition(0);
        motorI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motorI).setVelocity(velocity);
        motorI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (direction){
            motorI.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motorI.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    private void usingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}






