from cscore import CameraServer


def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    usb1 = cs.startAutomaticCapture(name="frontcam", path='/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0')

    usb2 = cs.startAutomaticCapture(name="othercam", path='/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0')

    # needs to be updated when other cam is installed
    # go to http://roborio-3172-frc.local:1181/
    cs.waitForever()
