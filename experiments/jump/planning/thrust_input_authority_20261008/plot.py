"""Plot saved native reference vs independently reconstructed sensor H."""
from pathlib import Path
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';O=D/'thrust_input_authority_20261008'
def read(p):return list(csv.DictReader(p.open()))
fig,axs=plt.subplots(4,2,figsize=(13,13))
for i,n in enumerate(['B1','B2','B3','T1']):
 rr=read(O/(n+'_matched.csv'));truth=read(D/'thrust_momentum_budget_20261008'/n/'steps.csv');start=next(int(r['sim_time_ns']) for r in truth if float(r['thrust_ms'])==0)
 x=[(int(r['sensor_stamp_ns'])-start)/1e6 for r in rr]
 axs[i,0].plot([float(r['thrust_ms']) for r in truth],[float(r['H_pitch']) for r in truth],label='native seven-body reference')
 axs[i,0].plot(x,[float(r['H_encoder_imu']) for r in rr],'o',ms=4,label='encoder + raw IMU reconstructed H')
 axs[i,0].set_ylabel(n+' H kg m2/s')
 axs[i,1].plot(x,[float(r['same_stamp_error']) for r in rr],'o-',label='same-stamp error')
 axs[i,1].plot(x,[float(r['current_time_error']) for r in rr],label='error if treated as control-time H')
 axs[i,1].set_ylabel(n+' H error kg m2/s')
 for ax in axs[i,:]:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('sensor timestamp ms from first THRUST log')
fig.suptitle('Saved-data validation only: sensor estimator vs native truth; no physical runs')
fig.tight_layout();fig.savefig(O/'H_observation_validation.png',dpi=120)
