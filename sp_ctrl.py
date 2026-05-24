
from rotary_sim import *


class DemoCtrl(Controller):

    def __init__(self):
        self.vfd_setpoint_hz: float = 0.0
        self.cvt_high_cmd: bool = False
        self.cvt_low_cmd: bool = False
        self.gearbox_backgear_active: bool = False

        self.old_setpoint = 0.0

        self.CVT_MAX =0.25
        self.CVT_MIN = 0.0975

        self.target_cvt_ratio = (self.CVT_MAX + self.CVT_MIN)*0.5

        self.np = 4

        self.ratio_estimate = 1.0

        self.step = 0

        self.op_cvt_setpoint = 0.17
        self.op_vfd_f = 0.0
        self.error_integral = 0.0


        self.step_log = []
        self.ratio_logs = []
        self.ratio_setp_logs = []
        self.sp_setpoint_logs = []

        self.t = 0.0

        self.timestamp = 0.0


    def calc_op_point(self, sp_setpoint):

        f_op_max = 50
        f_op_min = 50
        f_max = 70
        f_min = 10

        kg = 1
        n_p = 4

        # upper limit with f = 50
        n_upper = f_op_max * n_p * 60 * kg * self.CVT_MAX
        n_lower = f_op_min * n_p * 60 * kg * self.CVT_MIN

        n_max = f_max * n_p * 60 * kg * self.CVT_MAX
        n_min = f_min * n_p * 60 * kg * self.CVT_MIN

        kc = 0.17
        f = 0

        # Limit values of sp setpoint
        sp_setpoint = np.clip(sp_setpoint,0.0,n_max)

        if sp_setpoint > n_upper:
            kc = self.CVT_MAX
            f = sp_setpoint/(n_p*60*kg*kc)
        elif sp_setpoint > n_lower:
            f = 50
            kc = sp_setpoint/(n_p*60*kg*f)
        elif sp_setpoint > n_min:
            kc = self.CVT_MIN
            f = sp_setpoint/(n_p*60*kg*kc)
        else:
            f = 0.0


        return f,kc

    
    def ctrl_step(self, setpoint_rpm:float, spindle_act_rpm: float, cvt_at_max: bool, cvt_at_min: bool, vfd_at_speed:bool, dt:float):

        

        mtr_rpm = self.vfd_setpoint_hz * self.np * 60

        Kp = 0.0
        Ki = 0.0001

        if self.step == 0:
            # New cmd?

            if self.old_setpoint != setpoint_rpm:

                # Calc new ratio/VFD operating point
                self.op_vfd_f, self.op_cvt_setpoint  = self.calc_op_point(setpoint_rpm)
                self.step = 1
            else:

                # PI speed ctrl
                error = setpoint_rpm - spindle_act_rpm 
                self.error_integral += error 

                self.vfd_setpoint_hz = self.op_vfd_f + error*Kp + self.error_integral*Ki


            
        elif self.step == 1:


            # Set VFD to 10 Hz
            self.vfd_setpoint_hz = 10

            if vfd_at_speed:
                self.step = 2
                self.timestamp = self.t


        elif self.step == 2:
            # Control the CVT ratio
            ALPHA = 0.5
            self.ratio_estimate = self.ratio_estimate*ALPHA + (1-ALPHA)*spindle_act_rpm/mtr_rpm

            if self.timestamp > 0.5:

                cvt_error = self.op_cvt_setpoint - self.ratio_estimate

                TOL = 0.01
                self.cvt_low_cmd = cvt_error < -TOL
                self.cvt_high_cmd = cvt_error > TOL


            if abs(self.ratio_estimate - self.op_cvt_setpoint) < TOL:
                self.step = 3

                self.cvt_low_cmd = False
                self.cvt_high_cmd = False 

                self.vfd_setpoint_hz = self.op_vfd_f

                
        elif self.step == 3:

            if vfd_at_speed:
                self.step = 0
                self.error_integral = 0


        self.old_setpoint = setpoint_rpm

        # Log
        self.ratio_logs.append(self.ratio_estimate)
        self.step_log.append(self.step)
        self.ratio_setp_logs.append(self.op_cvt_setpoint)
        self.sp_setpoint_logs.append(setpoint_rpm)

        self.t += dt



def main():

    conf = get_default_sim_conf()
    
    sim = RotarySpindleSim(conf,0.01,30)

    ctrl = DemoCtrl()

    sim.sim(ctrl)

    sim.plot_results(prefix='CL_',save_figs=True)

    plt.figure()
    plt.plot(sim.t,ctrl.sp_setpoint_logs,'k--',label='Setpoint')
    plt.plot(sim.t,sim.sp_act,label='Actual')
    plt.ylabel('Spindle RPM')
    plt.xlabel('Time [s]')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'docs/CL_sp_ctrl.png')



    plt.figure()

    plt.plot(sim.t,ctrl.step_log)
    plt.title("ctrl-step")

    plt.grid(True)


    plt.figure()

    plt.plot(sim.t,ctrl.ratio_logs)
    plt.plot(sim.t,ctrl.ratio_setp_logs,'k--')
    plt.xlabel('Time [s]')
    plt.ylabel("$k_c$")
    plt.grid(True)

    plt.title("Ratio estimate")

    plt.savefig('docs/CL_ratio_estimate.png')

    plt.show()

    
if __name__ == '__main__':
    main()
