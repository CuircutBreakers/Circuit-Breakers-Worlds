package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Snail Bot Red One Gamepad", group = "TeleOp")
public class SnailBotRedSingleDriver extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor spindle;
    private DcMotorEx launcherLeft, launcherRight;

    private Limelight3A limelight;

    private NormalizedColorSensor intakeSensorLeft, intakeSensorRight;
    private DistanceSensor intakeDistanceLeft, intakeDistanceRight;

    private final float[] leftHsv = new float[3];
    private final float[] rightHsv = new float[3];

    private int launcherDirection = 1;

    private static final double AIM_TARGET_TX = 0.0;
    private static final double FAR_AIM_TARGET_TX = -2.5;
    private static final double AIM_DEADBAND_DEG = 1.0;
    private static final double AIM_KP = 0.025;
    private static final double AUTO_AIM_TRIGGER_DEADBAND = 0.05;

    private static final double A1 = 0.5;
    private static final double A2 = 1.2;
    private static final double A3 = 1.8;
    private static final double A4 = 2.5;

    private static final double Ratio  = 3.0 / 1.0;
    private static final double V_FAR  = 883 * Ratio;
    private static final double V_MID1 = 790 * Ratio;
    private static final double V_MID2 = 765 * Ratio;
    private static final double V_MID3 = 750 * Ratio;
    private static final double V_NEAR = 730 * Ratio;

    private static final double NO_TAG_BASE_VELOCITY = 750 * Ratio;
    private static final double VELOCITY_TRIM_STEP = 25;
    private double launchVelocityTrim = 0;

    private static final int SPINDLE_OPEN   = 140;
    private static final int SPINDLE_1BALL  = 50;
    private static final int SPINDLE_2BALL  = 10;

    private static final int SPINDLE_LAUNCH_1 = -75;
    private static final int SPINDLE_LAUNCH_2 = -250;
    private static final int SPINDLE_LAUNCH_3 = -340;

    private static final double SPINDLE_POWER = 1;

    private boolean lastA1 = false;
    private boolean lastB1 = false;
    private boolean lastX1 = false;
    private boolean lastRB1 = false;

    private int launchStage = -1;

    private enum SpindleMode {
        OPEN,
        ONE_BALL,
        TWO_BALL,
        LAUNCH_1,
        LAUNCH_2,
        LAUNCH_3
    }

    private SpindleMode spindleMode = SpindleMode.OPEN;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final double INTAKE_DEADBAND = 0.05;
    private static final double INTAKE_NORMAL_POWER = 1.0;
    private static final double INTAKE_PRIMED_POWER = 0.5;
    private boolean launcherPrimed = false;

    private static final double COLOR_CAP_CM = .65;

    private static final float GREEN_MIN = 100f;
    private static final float GREEN_MAX = 180f;

    private static final float PURPLE_MIN = 200f;
    private static final float PURPLE_MAX = 280f;

    private boolean artifactWasDetected = false;
    private int artifactCount = 0;

    private double zoneVelOut = 0.0;
    private double finalVelOut = 0.0;
    private double leftVelOut = 0.0;
    private double rightVelOut = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult limelightResult = getLimelightResult();
            double targetArea = getTargetArea(limelightResult);

            handleDrive(limelightResult);

            handleArtifactSensorSpindle();
            handleSpindle();

            handleLauncherPower(targetArea);

            handleIntake();
            updateTelemetry(limelightResult, targetArea);

            idle();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        intakeSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorLeft");
        intakeDistanceLeft = hardwareMap.get(DistanceSensor.class, "ColorSensorLeft");

        intakeSensorRight = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorRight");
        intakeDistanceRight = hardwareMap.get(DistanceSensor.class, "ColorSensorRight");

        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        spindle = hardwareMap.get(DcMotor.class, "spindle");
        spindle.setTargetPosition(SPINDLE_OPEN);
        spindle.setPower(SPINDLE_POWER);
        spindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
    }

    private LLResult getLimelightResult() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result;
        }

        return null;
    }

    private double getTargetArea(LLResult result) {
        if (result == null) return -1.0;
        return result.getTa();
    }

    private double pickVelocityFromTargetArea(double targetArea) {
        double baseVelocity;

        if (targetArea < 0) {
            baseVelocity = NO_TAG_BASE_VELOCITY;
        } else if (targetArea < A1) {
            baseVelocity = V_FAR;
        } else if (targetArea < A2) {
            baseVelocity = V_MID1;
        } else if (targetArea < A3) {
            baseVelocity = V_MID2;
        } else if (targetArea < A4) {
            baseVelocity = V_MID3;
        } else {
            baseVelocity = V_NEAR;
        }

        return Math.max(0, baseVelocity + launchVelocityTrim);
    }

    private double getAutoAimCorrection(LLResult result) {
        if (result == null) return 0.0;

        double targetArea = result.getTa();
        double targetTx = AIM_TARGET_TX;

        if (targetArea >= 0 && targetArea < A1) {
            targetTx = FAR_AIM_TARGET_TX;
        }

        double txError = result.getTx() - targetTx;

        if (Math.abs(txError) < AIM_DEADBAND_DEG) {
            return 0.0;
        }

        return txError * AIM_KP;
    }

    private void handleDrive(LLResult limelightResult) {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.left_trigger > AUTO_AIM_TRIGGER_DEADBAND && limelightResult != null) {
            rx = getAutoAimCorrection(limelightResult);
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        double drivePower = getDrivePowerModifier();

        frontLeft.setPower(fl * drivePower);
        backLeft.setPower(bl * drivePower);
        frontRight.setPower(fr * drivePower);
        backRight.setPower(br * drivePower);
    }

    private double getDrivePowerModifier() {
        if (gamepad1.left_bumper) return 0.25;
        return 0.7;
    }

    private void handleLauncherPower(double targetArea) {
        boolean upPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean downPressed = gamepad1.dpad_down && !lastDpadDown;

        if (upPressed) launchVelocityTrim += VELOCITY_TRIM_STEP;
        if (downPressed) launchVelocityTrim -= VELOCITY_TRIM_STEP;

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_left) {
            launcherDirection = -1;
        } else if (gamepad1.dpad_right) {
            launcherDirection = 1;
        }

        zoneVelOut = pickVelocityFromTargetArea(targetArea);
        finalVelOut = zoneVelOut;

        boolean fire = gamepad1.left_trigger > 0.05;
        launcherPrimed = fire;

        if (fire) {
            launcherLeft.setVelocity((finalVelOut * launcherDirection) * -1);
            launcherRight.setVelocity((finalVelOut * launcherDirection) * -1);
        } else {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }

        leftVelOut = launcherLeft.getVelocity();
        rightVelOut = launcherRight.getVelocity();
    }

    private void setSpindleTarget(int targetTicks, SpindleMode mode) {
        spindle.setTargetPosition(targetTicks);
        spindle.setPower(SPINDLE_POWER);
        spindleMode = mode;
    }

    private void handleArtifactSensorSpindle() {
        double leftDistance = intakeDistanceLeft.getDistance(DistanceUnit.CM);
        double rightDistance = intakeDistanceRight.getDistance(DistanceUnit.CM);

        boolean leftCloseEnough =
                !Double.isNaN(leftDistance) &&
                        leftDistance <= COLOR_CAP_CM;

        boolean rightCloseEnough =
                !Double.isNaN(rightDistance) &&
                        rightDistance <= COLOR_CAP_CM;

        String leftResult = "TOO FAR";
        String rightResult = "TOO FAR";

        if (leftCloseEnough) {
            leftResult = classifyColor(
                    intakeSensorLeft,
                    intakeDistanceLeft,
                    leftHsv
            );
        }

        if (rightCloseEnough) {
            rightResult = classifyColor(
                    intakeSensorRight,
                    intakeDistanceRight,
                    rightHsv
            );
        }

        boolean detected =
                leftResult.equals("GREEN") ||
                        leftResult.equals("PURPLE") ||
                        rightResult.equals("GREEN") ||
                        rightResult.equals("PURPLE");

        if (detected && !artifactWasDetected) {
            artifactCount++;

            if (artifactCount == 1) {
                setSpindleTarget(
                        SPINDLE_1BALL,
                        SpindleMode.ONE_BALL
                );

                launchStage = -1;

            } else if (artifactCount == 2) {
                setSpindleTarget(
                        SPINDLE_2BALL,
                        SpindleMode.TWO_BALL
                );

                launchStage = -1;
            }
        }

        artifactWasDetected = detected;

        if (gamepad1.b) {
            artifactCount = 0;
        }
    }

    private String classifyColor(
            NormalizedColorSensor color,
            DistanceSensor distance,
            float[] hsvOut
    ) {
        double d = distance.getDistance(DistanceUnit.CM);

        if (Double.isNaN(d) || d > COLOR_CAP_CM) {
            return "TOO FAR";
        }

        NormalizedRGBA c = color.getNormalizedColors();

        Color.RGBToHSV(
                (int)(c.red * 255),
                (int)(c.green * 255),
                (int)(c.blue * 255),
                hsvOut
        );

        float h = hsvOut[0];

        if (h >= GREEN_MIN && h <= GREEN_MAX) {
            return "GREEN";
        }

        if (h >= PURPLE_MIN && h <= PURPLE_MAX) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }

    private void handleIntake() {
        double intakePower = gamepad1.right_trigger;

        if (launcherPrimed) {
            if (intakePower > INTAKE_DEADBAND) {
                intake.setPower(intakePower * INTAKE_PRIMED_POWER);
            } else {
                intake.setPower(INTAKE_PRIMED_POWER);
            }

            return;
        }

        if (intakePower < INTAKE_DEADBAND) {
            intake.setPower(0);
        } else {
            intake.setPower(intakePower * INTAKE_NORMAL_POWER);
        }
    }

    private void handleSpindle() {
        boolean aPressed  = gamepad1.a && !lastA1;
        boolean bPressed  = gamepad1.b && !lastB1;
        boolean xPressed  = gamepad1.x && !lastX1;
        boolean rbPressed = gamepad1.right_bumper && !lastRB1;

        if (bPressed) {
            setSpindleTarget(SPINDLE_OPEN, SpindleMode.OPEN);
            launchStage = -1;

        } else if (aPressed) {
            setSpindleTarget(SPINDLE_1BALL, SpindleMode.ONE_BALL);
            launchStage = -1;

        } else if (xPressed) {
            setSpindleTarget(SPINDLE_2BALL, SpindleMode.TWO_BALL);
            launchStage = -1;

        } else if (rbPressed) {
            launchStage = (launchStage + 1) % 3;

            if (launchStage == 0) {
                setSpindleTarget(SPINDLE_LAUNCH_1, SpindleMode.LAUNCH_1);
            } else if (launchStage == 1) {
                setSpindleTarget(SPINDLE_LAUNCH_2, SpindleMode.LAUNCH_2);
            } else {
                setSpindleTarget(SPINDLE_LAUNCH_3, SpindleMode.LAUNCH_3);
            }
        }

        lastA1 = gamepad1.a;
        lastB1 = gamepad1.b;
        lastX1 = gamepad1.x;
        lastRB1 = gamepad1.right_bumper;
    }

    private void updateTelemetry(LLResult limelightResult, double targetArea) {
        telemetry.addData("Drive Power", getDrivePowerModifier());

        boolean targetVisible = limelightResult != null;
        telemetry.addData("Limelight Target", targetVisible ? "FOUND" : "NOT FOUND");

        if (targetVisible) {
            telemetry.addData("tx", "%.2f", limelightResult.getTx());
            telemetry.addData("ty", "%.2f", limelightResult.getTy());
            telemetry.addData("Target Area / Tag Size", "%.3f", targetArea);
            telemetry.addData("Auto Aim Correction", "%.3f", getAutoAimCorrection(limelightResult));

            if (targetArea >= 0 && targetArea < A1) {
                telemetry.addData("Aim Zone", "FAR - shifted right");
                telemetry.addData("Aim Target TX", "%.2f", FAR_AIM_TARGET_TX);
            } else {
                telemetry.addData("Aim Zone", "NORMAL");
                telemetry.addData("Aim Target TX", "%.2f", AIM_TARGET_TX);
            }
        } else {
            telemetry.addData("tx", "N/A");
            telemetry.addData("ty", "N/A");
            telemetry.addData("Target Area / Tag Size", "N/A");
            telemetry.addData("Auto Aim Correction", "N/A");
            telemetry.addData("Aim Zone", "N/A");
            telemetry.addData("Aim Target TX", "N/A");
        }

        telemetry.addData("ZoneVel Base + Trim(tps)", "%.0f", zoneVelOut);
        telemetry.addData("Launch Velocity Trim", "%.0f", launchVelocityTrim);
        telemetry.addData("FinalVel(tps)", "%.0f", finalVelOut);
        telemetry.addData("LeftVel(tps)", "%.0f", leftVelOut);
        telemetry.addData("RightVel(tps)", "%.0f", rightVelOut);

        telemetry.addData("Spindle Mode", spindleMode);
        telemetry.addData("Spindle Current (ticks)", "%d", spindle.getCurrentPosition());
        telemetry.addData("Spindle Target  (ticks)", "%d", spindle.getTargetPosition());

        telemetry.addData("Artifact Count", artifactCount);
        telemetry.addData("Artifact Detected", artifactWasDetected);
        telemetry.addData("Left Distance cm", "%.1f", intakeDistanceLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance cm", "%.1f", intakeDistanceRight.getDistance(DistanceUnit.CM));

        telemetry.update();
    }
}