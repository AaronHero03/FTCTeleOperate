package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {


    double kp, ki, kd;
    double out;
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.kp = Kp;
        this.ki = Ki;
        this.kd = Kd;
    }

    double last_error = 0;
    double derivative, integral;
    double error;
    ElapsedTime timer = new ElapsedTime();

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */

    public double update(double target, double state) {
        error = target - state;
        derivative = (error - last_error) / timer.seconds();
        integral = integral + (error * timer.seconds());

        out = (kp * error) + (ki * integral) + (kd * derivative);
        // PID logic and then return the output

        last_error = error;
        timer.reset();
        return out;
    }

    public double return_error(){
        return error;
    }
}