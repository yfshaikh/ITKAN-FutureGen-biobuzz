package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * HamzaTeli3 = HamzaTeli2 + FIELD-CENTRIC drive: pushing the stick away from the driver drives the
 * robot away from the driver, whichever way the robot is facing. Heading comes from the Pinpoint's gyro.
 * "Away" = the way the robot faced at INIT. Press OPTIONS to re-zero (point the robot away from you first).
 *
 * Hold R2: the flywheel holds TARGET_SHOOTER_VELOCITY; once the speed has stayed within
 * VELOCITY_TOLERANCE for STEADY_MS, the stopper opens, the transfer feeds, and the controller buzzes.
 * After each shot the speed dips, so "ready" goes false and the stopper closes until it's steady again.
 */
@TeleOp(name = "HamzaTeli3", group = "Competition")
public class HamzaTeli3 extends OpMode {


    private static final double TRANSFER_POWER = -1.0;
    private static final double REVERSE_POWER  = 1.0;


    private static final double TARGET_SHOOTER_VELOCITY = 2000.0; // Ticks per second


    private static final double FLOWER_UP_POS   = 1.0;
    private static final double FLOWER_DOWN_POS = 0.6;


    // Swapped: on the robot, 0.0 sat outside (not blocking) and 0.5 swung inside (blocking).
    // TUNE with ServoTuner: CLOSED must block the ball, OPEN must let it through.
    private static final double STOPPER_CLOSED_POS = 0.5;
    private static final double STOPPER_OPEN_POS   = 0.0;
    // TUNE: how close to the target counts as "at speed"
    private static final double VELOCITY_TOLERANCE = 100;
    // TUNE: how long it must stay at speed before we shoot
    private static final double STEADY_MS = 150;
    // TUNE: once open, the stopper stays open at least this long, so a small speed dip can't snap it shut
    private static final double STOPPER_MIN_OPEN_MS = 400;

    private static final boolean SWAP_BACK_MOTORS = false;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2;
    private Servo stopperServo;
    private GoBildaPinpointDriver pinpoint;   // only its gyro (heading) is used here

    private boolean flowerIsDown = false;


    // timer to track how long the speed has been steady for
    private ElapsedTime steadyTimer = new ElapsedTime();
    // keep track of if the shooter was ready (for the rumble on the controller)
    private boolean wasReady = false;

    // stopper: open/closed state and how long it has been open
    private boolean stopperOpen = false;
    private ElapsedTime stopperTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private String lastFlowerLog = "", lastStopperLog = "";

    @Override
    public void init() {

        frontLeft    = hardwareMap.get(DcMotor.class, "fl");
        frontRight   = hardwareMap.get(DcMotor.class, "fr");
        backLeft     = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "br" : "bl");
        backRight    = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "bl" : "br");
        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight = hardwareMap.get(DcMotorEx.class, "sr");
        flowerServo1 = hardwareMap.get(Servo.class, "f1");
        flowerServo2 = hardwareMap.get(Servo.class, "f2");
        stopperServo = hardwareMap.get(Servo.class, "ss");
        pinpoint     = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();   // robot must be still: heading 0 = the way it faces now

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);

        flowerServo1.setDirection(Servo.Direction.FORWARD);
        flowerServo2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setFlower(FLOWER_UP_POS);
        stopperServo.setPosition(STOPPER_CLOSED_POS);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
    }

    @Override
    public void loop() {

        double y  = deadband(gamepad1.left_stick_y);
        double x  = deadband(-gamepad1.left_stick_x);
        double rx = deadband(gamepad1.right_stick_x);

        // if the user clicks "options", we redo the orientation (whereever the robot is pointing is now "forward")
        pinpoint.update();
        if (gamepad1.options) {
            pinpoint.setHeading(0, AngleUnit.RADIANS);
        }
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);
        x = fieldX * 1.1;   // strafe correction goes AFTER the rotation (it fixes the robot's sideways motion)
        y = fieldY;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);


        // logic for right bumper/trigger, which controls the shooter
        // uses setShooterVelocity, works out "ready", and opens the stopper only while ready
        boolean ready = false;
        if (gamepad1.right_bumper) {
            setShooterPower(-1.0);
            stopperOpen = false;
        } else if (gamepad1.right_trigger > 0.1) {
            setShooterVelocity(TARGET_SHOOTER_VELOCITY);
            ready = shooterReady();
            if (ready && !stopperOpen) {
                stopperOpen = true;           // open, and start the minimum-open timer
                stopperTimer.reset();
            } else if (!ready && stopperOpen && stopperTimer.milliseconds() >= STOPPER_MIN_OPEN_MS) {
                stopperOpen = false;          // only close after it has been open long enough
            }
        } else {
            setShooterPower(0);
            stopperOpen = false;
        }
        stopperServo.setPosition(stopperOpen ? STOPPER_OPEN_POS : STOPPER_CLOSED_POS);


        // intake also feeds while shooter is ready, so holding R2 is enough to shoot.
        if (gamepad1.left_bumper) {
            setTransfer(REVERSE_POWER);
        } else if (gamepad1.left_trigger > 0.1 || stopperOpen) {
            setTransfer(TRANSFER_POWER);
        } else {
            setTransfer(0);
        }


        if (gamepad1.cross) {
            flowerIsDown = true;
        } else if (gamepad1.triangle) {
            flowerIsDown = false;
        }
        setFlower(flowerIsDown ? FLOWER_DOWN_POS : FLOWER_UP_POS);


        // driver feedback: rumble once when the shooter becomes ready; show READY on telemetry
        if (ready && !wasReady) {
            gamepad1.rumble(200);   // one short buzz the moment it's ready
        }


        wasReady = ready;

        // LOGGING. getPosition() is the position we last SENT, not where the servo really is:
        // if the numbers change but the part doesn't move, it's wiring/config/mechanical, not code.
        String flowerLog = String.format("cross=%b triangle=%b -> %s (sent %.2f)",
                gamepad1.cross, gamepad1.triangle, flowerIsDown ? "DOWN" : "UP", flowerServo1.getPosition());
        String stopperLog = String.format("ready=%b -> %s (sent %.2f)",
                ready, stopperOpen ? "OPEN" : "CLOSED", stopperServo.getPosition());
        // every change also goes to the robot log with a timestamp, so brief flickers aren't missed:
        //   adb logcat | grep HamzaTeli3
        if (!flowerLog.equals(lastFlowerLog)) {
            RobotLog.ii("HamzaTeli3", "%.2fs FLOWER %s", runtime.seconds(), flowerLog);
            lastFlowerLog = flowerLog;
        }
        if (!stopperLog.equals(lastStopperLog)) {
            RobotLog.ii("HamzaTeli3", "%.2fs STOPPER %s speed=%.0f", runtime.seconds(), stopperLog, shooterSpeed());
            lastStopperLog = stopperLog;
        }

        telemetry.addData("Heading (deg)", "%.1f   (OPTIONS = re-zero)", Math.toDegrees(heading));
        telemetry.addData("Shooter", ready ? "READY" : "spinning up");
        telemetry.addData("Shooter speed", shooterSpeed());
        telemetry.addData("Shooter L Vel", shooterLeft.getVelocity());
        telemetry.addData("Shooter R Vel", shooterRight.getVelocity());
        telemetry.addData("Flower", flowerLog);
        telemetry.addData("Flower Pos 1 / 2 (sent)", "%.2f / %.2f", flowerServo1.getPosition(), flowerServo2.getPosition());
        telemetry.addData("Stopper", stopperLog);
        telemetry.addData("Stopper open for", stopperOpen ? String.format("%.0f ms", stopperTimer.milliseconds()) : "-");
        telemetry.update();
    }



    private void setShooterVelocity(double ticksPerSecond) {
        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    // calculates shooter speed
    private double shooterSpeed() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }

    // This function returns true once the speed has stayed close to the target for STEADY_MS.
    private boolean shooterReady() {
        double error = Math.abs(shooterSpeed() - TARGET_SHOOTER_VELOCITY);
        if (error > VELOCITY_TOLERANCE) {
            steadyTimer.reset();   // not steady yet: start counting again
        }
        if (steadyTimer.milliseconds() >= STEADY_MS) {
            return true;
        } else {
            return false;
        }
    }

    private double deadband(double v) {
        return Math.abs(v) < 0.08 ? 0 : v;
    }

    private void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setFlower(double position) {
        flowerServo1.setPosition(position);
        flowerServo2.setPosition(position);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);}
}
