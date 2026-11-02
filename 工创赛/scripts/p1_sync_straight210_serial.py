import p1_sync_serial_motion as serial_motion
from p1_straight210_policy import make_controller
Straight210Controller=make_controller(serial_motion)
def main():
    serial_motion.Controller=Straight210Controller
    return serial_motion.main()
if __name__=='__main__':raise SystemExit(main())
