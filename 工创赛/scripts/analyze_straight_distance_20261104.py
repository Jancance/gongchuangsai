"""Offline ground-distance review; never modifies firmware or opens a device."""
import argparse,json,math,statistics
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--measured-mm',type=float,nargs='+',required=True,help='Same chassis point ground forward displacements, not encoder distances')
 p.add_argument('--target-mm',type=float,default=2100)
 p.add_argument('--current-scale',type=float,default=1.02182540)
 a=p.parse_args()
 if not all(math.isfinite(v) and v>0 for v in [*a.measured_mm,a.target_mm,a.current_scale]):p.error('all inputs must be finite and positive')
 mean=statistics.mean(a.measured_mm)
 print(json.dumps(dict(measured_mm=a.measured_mm,target_mm=a.target_mm,mean_measured_mm=mean,
  errors_mm=[v-a.target_mm for v in a.measured_mm],mean_error_mm=mean-a.target_mm,
  mean_error_percent=(mean/a.target_mm-1)*100,range_mm=max(a.measured_mm)-min(a.measured_mm),
  current_progress_scale=a.current_scale,candidate_progress_scale=a.current_scale*mean/a.target_mm,
  formula='new_progress_scale = old_progress_scale * ground_measured / target',
  candidate_only=True,firmware_modified=False,
  repeatability_still_needs_review=len(a.measured_mm)<3,
  note='Check lateral drift, slip and speed/load dependence before applying a scalar.'),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
