import wpilib
import wpilib.drive
import time
from wpilib.drive import DifferentialDrive

class Apollo(wpilib.TimedRobot):

    def __init__(self):
        super().__init__()
        self.joystick = wpilib.XboxController(0)

        self.left_drive_motor = wpilib.Talon(0)
        self.left_drive_motor_2 = wpilib.Talon(1)
        self.right_drive_motor = wpilib.Talon(2)
        self.right_drive_motor_2 = wpilib.Talon(3)
        
        self.left_drive_motor_group = wpilib.SpeedControllerGroup(self.left_drive_motor, self.left_drive_motor_2)
        self.right_drive_motor_group = wpilib.SpeedControllerGroup(self.right_drive_motor, self.right_drive_motor_2)

        self.drive = DifferentialDrive(self.left_drive_motor_group, self.right_drive_motor_group)
        self.drive_rev = False

        self.lift_motor = wpilib.Spark(4)
        self.lift_motor_2 = wpilib.Spark(5)
        self.lift_motor_group = wpilib.SpeedControllerGroup(self.lift_motor, self.lift_motor_2)
        self.lift_motor_speed = 0
        self.lock_controls = False

        self.front_motor = wpilib.Spark(6)
        self.front_motor_2 = wpilib.Spark(7)
        self.front_motor_2.setInverted(True)
        self.front_motor_group = wpilib.SpeedControllerGroup(self.front_motor, self.front_motor_2)

        self.hatch_solenoid = wpilib.DoubleSolenoid(0, 1)

        self.encoder = wpilib.Encoder(aChannel=0, bChannel=1)

    def reset(self):
        self.drive_rev = False

        while self.lift_motor_speed > 0:
            self.lift_motor_speed -= 0.002
            self.lift_motor_group.set(self.lift_motor_speed)

        if self.lift_motor_speed < 0:
            self.lift_motor_speed = 0
            self.lift_motor_group.set(self.lift_motor_speed)

        self.front_speed = 0
        self.front_motor_group.set(self.front_speed)
        self.lock_controls = False
        self.hatch_solenoid.set(0)
        self.encoder.reset()

    def drive_control(self):
        lh_y = self.joystick.getY(self.joystick.Hand.kLeft) * (1 / 2)
        rh_x = self.joystick.getX(self.joystick.Hand.kRight) * (1 / 2)

        if self.joystick.getStickButtonPressed(self.joystick.Hand.kLeft) or \
           self.joystick.getStickButtonPressed(self.joystick.Hand.kRight):
            self.drive_rev = not self.drive_rev

        if self.drive_rev:
            self.drive.arcadeDrive(lh_y, rh_x, True)
        else:
            self.drive.arcadeDrive(lh_y*-1, rh_x, True)
        
    def lift_control(self):
        low_volt = 0.2
        if self.joystick.getYButtonPressed():
            self.lock_controls = not self.lock_controls

        if not self.lock_controls:
            if self.joystick.getTriggerAxis(self.joystick.Hand.kLeft) > 0.9:
                # self.lift_motor_speed = 0.6
                if self.lift_motor_speed == 0.0 or self.lift_motor_speed == low_volt:
                    self.lift_motor_speed = 0.6
                elif int(abs(self.encoder.getRate())) == 0:
                    self.lift_motor_speed += 0.01
            elif (self.joystick.getTriggerAxis(self.joystick.Hand.kRight) > 0.9)\
                    and self.joystick.getBButton():
                self.lift_motor_speed -= 0.01 if self.lift_motor_speed > 0.0 else 0.0
            elif self.joystick.getTriggerAxis(self.joystick.Hand.kRight) > 0.9:
                self.lift_motor_speed = low_volt
            elif (self.lift_motor_speed > low_volt and abs(self.encoder.getRate()) > 0):
                self.lift_motor_speed -= 0.0025
            elif (0.0 < self.lift_motor_speed < low_volt):
                self.lift_motor_speed -= 0.0025 if self.lift_motor_speed > 0.0 else 0.00
            elif self.lift_motor_speed < 0:
                self.lift_motor_speed = 0

            self.lift_motor_group.set(self.lift_motor_speed)

    def grab_control(self):
        if self.joystick.getBumper(self.joystick.Hand.kLeft) and not \
           self.joystick.getBumper(self.joystick.Hand.kRight):
            self.front_motor_group.set(0.33)

        elif self.joystick.getBumperPressed(self.joystick.Hand.kRight) and not \
           self.joystick.getBumper(self.joystick.Hand.kLeft):
            self.front_motor_group.set(-0.7)

        elif self.joystick.getBumperReleased(self.joystick.Hand.kLeft) or \
           self.joystick.getBumperReleased(self.joystick.Hand.kRight):
            self.front_motor_group.set(0)

    def hatch_control(self):
        if self.joystick.getXButtonPressed():
            self.hatch_solenoid.set(2)
        elif self.joystick.getXButtonReleased():
            self.hatch_solenoid.set(1)

    def robotInit(self):
        self.reset()
        wpilib.CameraServer.launch('vision.py:main')

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        self.reset()

    def autonomousPeriodic(self):
        self.drive_control()
        self.lift_control()
        self.grab_control()
        self.hatch_control()

    def disabledInit(self):
        self.reset()

    def disabledPeriodic(self):
        self.reset()

    def teleopInit(self):
        self.reset()
        self.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.drive_control()
        self.lift_control()
        self.grab_control()
        self.hatch_control()


if __name__ == '__main__':
    wpilib.run(Apollo)
