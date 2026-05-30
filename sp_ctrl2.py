
from rotary_sim import *


class DemoCtrl(Controller):

    def __init__(self):
        self.vfd_setpoint_hz: float = 0.0
        self.cvt_high_cmd: bool = False
        self.cvt_low_cmd: bool = False
        self.gearbox_backgear_active: bool = False


        self.vfd_rate_limit = 60/10

        self.old_setpoint = 0.0

        self.CVT_MAX =0.25
        self.CVT_MIN = 0.0975

        self.target_cvt_ratio = (self.CVT_MAX + self.CVT_MIN)*0.5

        self.n_p = 4

        self.ratio_estimate = 0.0

        self.speed_est = 0.0

        self.step = 0

        self.op_cvt_setpoint = 0.17
        self.op_vfd_f = 0.0
        self.error_integral = 0.0


        self.step_log = []
        self.ratio_logs = []
        self.mtr_speed_est_logs = []
        self.ratio_setp_logs = []
        self.sp_setpoint_logs = []
        self.sp_act_est_logs = []

        self.t = 0.0

        self.timestamp = 0.0


        self.last_cvt_cmd_timestamp = 0.0


    def calc_op_point(self, sp_setpoint):

        f_op_max = 50
        f_op_min = 50
        f_max = 70
        f_min = 10

        kg = 1
        n_p = self.n_p

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

        mtr_rpm = self.vfd_setpoint_hz * self.n_p * 60

        Kp = 0.0
        Ki = 0.0001

        # Deafult
        self.cvt_low_cmd = False
        self.cvt_high_cmd = False 

        sp_est = self.speed_est*self.n_p*60*self.op_cvt_setpoint

        if self.step == 0:
            # New cmd?

            if self.old_setpoint != setpoint_rpm and setpoint_rpm > 0.0:

                # Calc new ratio/VFD operating point
                self.op_vfd_f, self.op_cvt_setpoint  = self.calc_op_point(setpoint_rpm)
                self.step = 1
                self.last_cvt_cmd_timestamp = self.t
            else:

                # PI speed ctrl
                error = setpoint_rpm - spindle_act_rpm 
                self.error_integral += error 

                self.vfd_setpoint_hz = self.op_vfd_f + error*Kp + self.error_integral*Ki


            
        elif self.step == 1:
            # ramping to target vel active

            # Set VFD to 10 Hz
            self.vfd_setpoint_hz = self.op_vfd_f

            # during ramp, adjust CVT
            speed_error = sp_est - spindle_act_rpm
            true_error = setpoint_rpm - spindle_act_rpm
            TOL = 50
            if abs(speed_error) < TOL:
                speed_error = 0.0

            if speed_error > 0.0 and not cvt_at_max:
                self.cvt_high_cmd = True
                self.last_cvt_cmd_timestamp = self.t

            if speed_error < 0.0 and not cvt_at_min:
                self.cvt_low_cmd = True
                self.last_cvt_cmd_timestamp = self.t


           #if self.last_cvt_cmd_timestamp > 0.5 and abs(true_error) <= setpoint_rpm*0.5:
            if abs(speed_error) < TOL and abs(true_error) <= setpoint_rpm*0.5:
                self.step = 2
                self.timestamp = self.t
            
        elif self.step == 2:
            # Wait for VFD to finish ramp
            # maybe start PI control here already?

            if vfd_at_speed:
                self.step = 0

        
        # Include ramp before ratio est
        upper_lim = self.speed_est + self.vfd_rate_limit*dt 
        lower_lim = self.speed_est - self.vfd_rate_limit*dt 
        self.speed_est = np.clip(self.vfd_setpoint_hz, lower_lim,upper_lim)

        ALPHA = 0.9
        if mtr_rpm > 0:
            self.ratio_estimate = self.ratio_estimate*ALPHA + (1-ALPHA)*spindle_act_rpm/mtr_rpm

        self.old_setpoint = setpoint_rpm

        # Log
        self.ratio_logs.append(self.ratio_estimate)
        self.step_log.append(self.step)
        self.ratio_setp_logs.append(self.op_cvt_setpoint)
        self.sp_setpoint_logs.append(setpoint_rpm)
        self.mtr_speed_est_logs.append(self.speed_est)
        self.sp_act_est_logs.append(sp_est)

        self.t += dt


def main():

    conf = get_default_sim_conf()
    
    sim = RotarySpindleSim(conf,0.01,30)
    sim.p_c[0] = 1.0
    prefix = 'CL22_'
    save_figs = False

    ctrl = DemoCtrl()

    sim.sim(ctrl)

    sim.plot_results(prefix=prefix,save_figs=save_figs)

    plt.figure()
    plt.plot(sim.t,ctrl.sp_setpoint_logs,'k--',label='Setpoint')
    plt.plot(sim.t,sim.sp_act,label='Actual')
    plt.ylabel('Spindle RPM')
    plt.xlabel('Time [s]')
    plt.legend()
    plt.grid(True)
    if save_figs:
        plt.savefig(f'docs/{prefix}_sp_ctrl.png')

    plt.figure()

    plt.plot(sim.t,ctrl.step_log)
    plt.title("ctrl-step")

    plt.grid(True)

    plt.figure()

    plt.plot(sim.t,ctrl.sp_setpoint_logs,'k--',label='Setpoint')
    plt.plot(sim.t,sim.sp_act,label='Actual')
    plt.plot(sim.t,ctrl.sp_act_est_logs,label='Estimated')
    plt.xlabel('Time [s]')
    plt.legend()

    if save_figs:
        plt.savefig(f'docs/{prefix}speed_estimate.png')



    plt.show()

    
if __name__ == '__main__':
    main()
