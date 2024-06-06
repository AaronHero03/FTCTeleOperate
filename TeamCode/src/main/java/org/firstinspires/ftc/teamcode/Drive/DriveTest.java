package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class DriveTest extends LinearOpMode {

    DcMotor FrontR;
    DcMotor FrontL;
    DcMotor BackR;
    DcMotor BackL;
    IMU imu;

    @Override
    public void runOpMode(){

        DcMotor FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        DcMotor FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        DcMotor BackR = hardwareMap.get(DcMotor.class, "BackL");
        DcMotor BackL = hardwareMap.get(DcMotor.class, "BackR");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        while(opModeIsActive()){
            double M0 = - gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double M1 = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double M2 = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double M3 = - gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;

            FrontR.setPower(M0);
            FrontL.setPower(M1);
            BackR.setPower(M2);
            BackL.setPower(M3);

            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();

        }

    }
}
