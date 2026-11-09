"""Calculate a measurement candidate; never changes firmware or accesses motors."""
import argparse,json
def calculate(x0,x1,encoder0,encoder1,raw0,raw1,command=1250):
 delta=(encoder1-encoder0+32768)%65536-32768
 angle=abs(delta)*360/65536;travel=abs(x1-x0)
 if travel<=0 or angle<=0:raise ValueError('Need nonzero ruler travel and encoder movement')
 return dict(x0_mm=x0,x1_mm=x1,travel_mm=travel,encoder_delta=delta,motor_angle_deg=angle,
             command_units=command,command_units_per_mm=command/travel,
             motor_deg_per_mm=angle/travel,feedback_raw_per_degree=abs(raw1-raw0)/angle,
             virtual_3200_pulses_per_mm=angle/360*3200/travel,
             candidate_only=True,firmware_changed=False)
def main():
 p=argparse.ArgumentParser();p.add_argument('--x0',type=float,required=True);p.add_argument('--x1',type=float,required=True)
 p.add_argument('--encoder0',type=int,required=True);p.add_argument('--encoder1',type=int,required=True)
 p.add_argument('--raw0',type=int,required=True);p.add_argument('--raw1',type=int,required=True);a=p.parse_args()
 print(json.dumps(calculate(a.x0,a.x1,a.encoder0,a.encoder1,a.raw0,a.raw1),indent=2))
if __name__=='__main__':main()
