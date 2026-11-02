import p1_fine_serial_motion as serial_motion
from p1_straight180_policy import make_controller
Straight180Controller=make_controller(serial_motion)
def main():
    serial_motion.Controller=Straight180Controller
    return serial_motion.main()
if __name__=='__main__':raise SystemExit(main())
