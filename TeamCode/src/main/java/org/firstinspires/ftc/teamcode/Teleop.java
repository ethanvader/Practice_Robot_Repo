package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Johnny6Teleop")
public class Teleop extends OpMode {
    Johnny6 testrobo;

    //allows for driver customisation
    static final double STRAFE_FACTOR=1.1;

    @Override
    public void init(){
        johnny6=new Johnny6(this, Johnny6.Drivetrain.JOHNNY6);
    }

    @Override
    public void loop(){
        //telemetry.addData("front left encoder: ",johnny6.motorFrontLeft.getCurrentPosition());
        //telemetry.addData("back left encoder: ",johnny6.motorBackLeft.getCurrentPosition());
        //telemetry.addData("front right encoder: ",johnny6.motorFrontRight.getCurrentPosition());
        //telemetry.addData("back right encoder: ",johnny6.motorBackRight.getCurrentPosition());

        //telemetry.update();

        //Key variables for movement
        double y=-gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x;
        y *= y;
        if(gamepad1.left_stick_y > 0) {
            y = -y;
        }
        x *= x;
        if(gamepad1.left_stick_x < 0) {
            x = -x;
        }
        //double x=(gamepad1.left_stick_x*STRAFE_FACTOR)/2;
        double turn=gamepad1.right_stick_x/2;

        johnny6.move(x,y,turn);

        if(gamepad2.right_trigger>0) {
            testrobo.moveArm(gamepad2.right_trigger*.5);
        } else if(gamepad2.left_trigger > 0) {
            testrobo.moveArm(-gamepad2.left_trigger*.5);
        } else {
            johnny7.moveClaw(0);
        }

    }
}

