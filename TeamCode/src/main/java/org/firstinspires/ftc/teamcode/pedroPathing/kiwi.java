package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class kiwi extends Drivetrain {
    private IMU imu;
    DcMotorEx frente;
    DcMotorEx direita;
    DcMotorEx esquerda;



    double m1;
    double m2;
    double m3;

    double velocidade_motor1_eixoX;
    double velocidade_motor2_eixoX;
    double velocidade_motor3_eixoX;

    double velocidade_motor1_eixoY;
    double velocidade_motor2_eixoY;
    double velocidade_motor3_eixoY;


    public kiwi(HardwareMap hardwareMap){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);

        frente = hardwareMap.get(DcMotorEx.class, "frente");
        direita = hardwareMap.get(DcMotorEx.class, "direita");
        esquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
    }
    public void loop() {

    }
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {

        double anguloIMU = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double transX = correctivePower.getXComponent() + pathingPower.getXComponent();
        double transY = correctivePower.getXComponent() + pathingPower.getYComponent();
        double omega;
        omega = headingPower.getXComponent();
        double calculateDriveX = transX * Math.cos(robotHeading) + transY * Math.sin(robotHeading);
        double calculateDriveY = -transY * Math.sin(anguloIMU) + transY * Math.cos(anguloIMU);
        m1 = -calculateDriveY + omega;
        m2 = -(Math.sqrt(3) / 2.0) * calculateDriveX + 0.5 * calculateDriveY + omega;
        m3 = (Math.sqrt(3) / 2.0) * calculateDriveX + 0.5 * calculateDriveY + omega;

        //Define o máximo de velocidade dos motores como 1.0
        return new double[]{m1, m2, m3};
    }


    @Override
    public void updateConstants() {

    }

    @Override
    public void breakFollowing() {
        frente.setZeroPowerBehavior(BRAKE);
        direita.setZeroPowerBehavior(BRAKE);
        esquerda.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void runDrive(double[] drivePowers) {
        frente.setPower(drivePowers[0]);
        direita.setPower(drivePowers[1]);
        esquerda.setPower(drivePowers[2]);
    }

    @Override
    public void startTeleopDrive() {

    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {

    }

    @Override
    public double xVelocity() {
        velocidade_motor1_eixoX = 1;
        velocidade_motor2_eixoX = 1;
        velocidade_motor3_eixoX = 1;
        return 0;
    }

    @Override
    public double yVelocity() {
        velocidade_motor1_eixoY = 1;
        velocidade_motor2_eixoY = 1;
        velocidade_motor3_eixoY = 1;

        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {
        frente.setVelocity(velocidade_motor1_eixoX);
        direita.setVelocity(velocidade_motor2_eixoX);
        esquerda.setVelocity(velocidade_motor3_eixoX);
    }

    @Override
    public void setYVelocity(double yMovement) {
        frente.setVelocity(velocidade_motor1_eixoY);
        direita.setVelocity(velocidade_motor2_eixoY);
        esquerda.setVelocity(velocidade_motor3_eixoY);
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public String debugString() {
        return "Erro";
    }

    protected double[] maxPower(int i) {
        double maxMotor = Math.max(Math.abs(m1), Math.max(Math.abs(m2), Math.abs(m3)));
        if (maxMotor > i) {
            m1 /= maxMotor;
            m2 /= maxMotor;
            m3 /= maxMotor;
        }
        return new double[]{m1, m2, m3};
    }
}
