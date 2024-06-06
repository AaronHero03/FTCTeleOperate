    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;

    @Disabled
    @TeleOp
    public class PruebaCore extends LinearOpMode {

        DcMotor Arm;

        @Override
        public void runOpMode() throws InterruptedException {

            Arm = hardwareMap.get(DcMotor.class, "Arm");
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setTargetPosition(0);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx)Arm).setVelocity(100);
            Arm.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();
            if (isStopRequested()) return;

            while(opModeIsActive()){
                if(gamepad1.y){
                    Arm.setTargetPosition(125);
                }
                /*
                else if(gamepad1.a){
                    Arm.setTargetPosition(0);
                }
                */
                telemetry.addData("Arm: ", Arm.getCurrentPosition());
                telemetry.update();
            }
        }
    }
