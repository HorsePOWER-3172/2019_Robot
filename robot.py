import wpilib
import wpilib.drive
from robotpy_ext.control.toggle import Toggle
from wpilib.drive import DifferentialDrive


class Apollo(wpilib.TimedRobot):

    def robotInit(self):
        self.xbox = wpilib.XboxController(0)
        self.lift_lock = Toggle(self.xbox, 4, 0.5)

        self.left_drive_motor_group = wpilib.SpeedControllerGroup(wpilib.Talon(0), wpilib.Talon(1))
        self.right_drive_motor_group = wpilib.SpeedControllerGroup(wpilib.Talon(2), wpilib.Talon(3))

        self.drive = DifferentialDrive(self.left_drive_motor_group, self.right_drive_motor_group)
        self.drive_rev = False
        self.speedRatio = 0.5

        self.lift_motor = wpilib.SpeedControllerGroup(wpilib.Spark(4), wpilib.Spark(5))
        self.lift_motor_speed = 0.0

        # ball grab motors, need to spin in opposite directions
        self.front_motor_1 = wpilib.Spark(6)
        self.front_motor_2 = wpilib.Spark(7)
        self.front_motor_2.setInverted(True)
        self.front_motor = wpilib.SpeedControllerGroup(self.front_motor_1, self.front_motor_2)

        self.hatch_solenoid = wpilib.DoubleSolenoid(0, 1)  # pneumatic channels 0 & 1

        self.encoder = wpilib.Encoder(aChannel=0, bChannel=1)  # DIO 0 & 1
        wpilib.CameraServer.launch('vision.py:main')  # setup cameras: usb1 & usb2

        self.xbox_axis = {}  # init xbox axis dict
        self.debug = False  # enable to print debug info
        self.reset()  # i.e. ensure defaults are set

        self.loops = 0  # counter for program loops
        self.timer = wpilib.Timer()  # init timer

    def reset(self):
        # safely lower lift
        while self.lift_motor_speed > 0.0:
            self.lift_motor_speed -= 0.002 if self.lift_motor_speed > 0.0 else 0.0
            self.lift_motor.set(self.lift_motor_speed)

        self.front_motor.set(0)
        self.hatch_solenoid.set(0)
        self.encoder.reset()

        # init xbox axis info
        for i in range(0, self.xbox.getAxisCount()):
            self.xbox_axis[i] = self.xbox.getRawAxis(i)

    def drive_control(self):
        if self.xbox.getAButtonPressed():
            if self.speedRatio == 2/3:
                self.speedRatio = 0.5
            elif self.speedRatio == 0.5:
                self.speedRatio = 2/3

        lh_y = self.xbox.getY(self.xbox.Hand.kLeft) * self.speedRatio  # incr speed from 1/2 to 2/3
        rh_x = self.xbox.getX(self.xbox.Hand.kRight) * 2/3

        if self.xbox.getStickButtonPressed(self.xbox.Hand.kLeft) \
                or self.xbox.getStickButtonPressed(self.xbox.Hand.kRight):
            self.drive_rev = not self.drive_rev

        if self.drive_rev:
            self.drive.arcadeDrive(lh_y, rh_x, True)
        else:
            self.drive.arcadeDrive(lh_y*-1, rh_x, True)

    def button_status(self):
        # output to logs button or axis # and value
        for i in range(0, self.xbox.getAxisCount()):
            cur_axis = self.xbox.getRawAxis(i)
            if self.xbox_axis[i] != cur_axis:
                self.xbox_axis[i] = cur_axis
                self.logger.info("Axis {} = {}".format(i, cur_axis))
        for j in range(1, self.xbox.getButtonCount()+1):
            if self.xbox.getRawButtonPressed(j):
                self.logger.info("Button {} pressed".format(j))

    def lift_control(self):
        high_volt = 0.9         # volt high limit
        init_volt = 0.45        # voltage to start raising
        low_volt = 0.2          # volt low limit
        volt_rate_raise = 0.01  # step voltage down
        volt_rate_low = 0.005   # step voltage up

        def trigger_pressed(hand):
            # wrapper case to detect a Xbox Trigger hard press
            return self.xbox.getTriggerAxis(hand) > 0.9

        def auto_lower():
            # redundant logic to reach max holding voltage while falling
            if (round(self.lift_motor_speed,2) > low_volt) and (int(abs(self.encoder.getRate())) > 0):
                self.lift_motor_speed -= volt_rate_low
            # when voltage is below lower limit, then slowly reduce to zero
            # slow fall to bottom out
            elif (0.0 < round(self.lift_motor_speed,2) < low_volt) and (int(abs(self.encoder.getRate())) == 0):
                self.lift_motor_speed -= volt_rate_low if round(self.lift_motor.get(),2) > 0.0 else 0.00
            # fail-safe for when the speed goes negative
            # shouldn't happen, but was left in case
            elif self.lift_motor_speed < 0:
                self.lift_motor_speed = 0

        if self.lift_lock.on:
            # disable trigger input and allow the voltage to reduce to holding state
            auto_lower()
        else:
            # Left Trigger press
            if trigger_pressed(self.xbox.Hand.kLeft):
                # when starting, or below our low volt limit, then set speed to initial ramp voltage
                if (self.lift_motor_speed is 0.0) or (self.lift_motor_speed <= low_volt):
                    self.lift_motor_speed = init_volt
                # while the lift is not moving, slowly increase the voltage
                # don't pass high voltage limit
                elif (int(abs(self.encoder.getRate())) == 0) and (self.lift_motor_speed <= high_volt):
                    self.lift_motor_speed += volt_rate_raise
            # Right Trigger press
            # with B button held, lower to 0 volts
            elif trigger_pressed(self.xbox.Hand.kRight) and self.xbox.getBButton():
                self.lift_motor_speed -= volt_rate_low if self.lift_motor.get() > 0.0 else 0.0
            # while voltage is positive, then lower at lower rate, then bottom out at low volt limit
            elif trigger_pressed(self.xbox.Hand.kRight) and (self.lift_motor_speed > 0):
                self.lift_motor_speed -= volt_rate_low if round(self.lift_motor.get(),2) > low_volt else low_volt
            # no trigger pressed, then run auto set code
            # will either keep lowest holding voltage or lower to zero
            else:
                auto_lower()
            # set lift motor speed
            self.lift_motor.set(self.lift_motor_speed)

    def grab_control(self):
        if self.xbox.getBumper(self.xbox.Hand.kLeft) \
                and not self.xbox.getBumper(self.xbox.Hand.kRight):
            self.front_motor.set(0.33)
        elif self.xbox.getBumperPressed(self.xbox.Hand.kRight) \
                and not self.xbox.getBumper(self.xbox.Hand.kLeft):
            self.front_motor.set(-0.7)
        elif self.xbox.getBumperReleased(self.xbox.Hand.kLeft) \
                or self.xbox.getBumperReleased(self.xbox.Hand.kRight):
            self.front_motor.set(0)

    def hatch_control(self):
        if self.xbox.getXButtonPressed():
            self.hatch_solenoid.set(2)
        elif self.xbox.getXButtonReleased():
            self.hatch_solenoid.set(1)

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        self.reset()

    def autonomousPeriodic(self):
        self.drive_control()
        self.lift_control()
        self.grab_control()
        self.hatch_control()
        if self.debug:
            self.button_status()

    def disabledInit(self):
        self.reset()

    def disabledPeriodic(self):
        self.reset()

    def teleopInit(self):
        self.reset()
        self.drive.setSafetyEnabled(True)
        self.loops = 0
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        self.drive_control()
        self.lift_control()
        self.grab_control()
        self.hatch_control()
        if self.debug:
            self.button_status()


if __name__ == '__main__':
    wpilib.run(Apollo)
