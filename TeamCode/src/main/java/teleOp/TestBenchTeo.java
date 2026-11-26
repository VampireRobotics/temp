package teleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import hardwareParts.absoluteEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TestMotorTeo")
public class TestBenchTeo {
    private boolean opModeIsActive() {
        return true;
    }

    private boolean isStarted() {
        return true;
    }

    private boolean isStopRequested() {
        return true;
    }

    AnalogInput encoder;
    NormalizedColorSensor test_color;
    DcMotor motor1;
    Servo servo1;
    Servo servo2;
    TouchSensor touch;
    DistanceSensor distance;


    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motorIntake");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        encoder = hardwareMap.get(AnalogInput.class, "absoluteencoder");


        telemetry.addData("Connection", motor1.getConnectionInfo());
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
        }

        while (opModeIsActive()) {

            double voltage = encoder.getVoltage();
            double angle = voltage / 3.3 * 360.0;

            if (gamepad1.dpad_up) {
                servo2.setPosition(1.0);
            } else if (gamepad1.dpad_down) {
                servo2.setPosition(0.5);
            } else if (gamepad1.dpad_right) {
                servo2.setPosition(0.0);
            }
            if (gamepad1.triangle) {
                servo1.setPosition(1.0);
            } else if (gamepad1.square) {
                servo1.setPosition(0.5);
            } else if (gamepad1.circle) {
                servo1.setPosition(0.0);
            }
            if (gamepad1.left_trigger > 0.5) {
                motor1.setPower(1.0);
            } else if (gamepad1.right_trigger > 0.5) {
                motor1.setPower(-1.0);
            } else if (gamepad1.right_bumper) {
                motor1.setPower(0.0);
            }

            // touch sensor
            if (touch.isPressed()) {
                motor1.setPower(0.0);
                servo1.setPosition(0.0);
                servo2.setPosition(0.0);
            }
            //distance sensor
            if (distance.getDistance(DistanceUnit.CM) < 1.0) {
                motor1.setPower(0.8);
            }

            //color sensor
            NormalizedRGBA colors = test_color.getNormalizedColors();

            float sum = colors.red + colors.green + colors.blue;

            float r = colors.red / sum;
            float g = colors.green / sum;
            float b = colors.blue / sum;

            telemetry.addData("Raw Red", "%.3f", colors.red);
            telemetry.addData("Raw Green", "%.3f", colors.green);
            telemetry.addData("Raw Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData("Norm Red", "%.3f", r);
            telemetry.addData("Norm Green", "%.3f", g);
            telemetry.addData("Norm Blue", "%.3f", b);

            if (r > g && r > b) {
                telemetry.addData("Detected", "RED");
            } else if (g > r && g > b) {
                telemetry.addData("Detected", "GREEN");
            } else if (b > r && b > g) {
                telemetry.addData("Detected", "BLUE");
            } else {
                telemetry.addData("Detected", "UNKNOWN");
            }
            telemetry.update();
            if (colors.red > colors.green && colors.red > colors.blue) {
                servo1.setPosition(0.2);

            }
            if (colors.green > colors.blue && colors.green > colors.red) {
                servo1.setPosition(0.5);

            }
            if (colors.blue > colors.green && colors.blue > colors.red) {
                servo1.setPosition(1.0);




            }

            telemetry.update();
        }

    }

}
