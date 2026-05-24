import numpy as np 
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass
class SimConfig:

    # VFD Parameters
    vfd_max_hz:float # max Hz VFD
    vfd_min_hz:float # min Hz VFD

    hz_2_rps:float # no of polepairs, i.e. rotations per second = hz * hz_2_rps 
    vfd_ramp_time_s:float # ramp time in VFD in second (time to reach max Hz)
    vfd_tau_s:float # Time constant for VFD to reach setpoint (PT1-filter)

    # CVT Parameters
    k_cvt_high:float # Ratio for p_cvt = 1
    k_cvt_low:float # Ratio for p_cvt = 0
    cvt_speed_tau_s:float # time constant for positioning CVT

    # Fixed gearbox
    k_gear_high: float # Fixed gearbox high gear output/input
    k_gear_low:float # * low gear *

    

@runtime_checkable
class Controller(Protocol):
    """
    Protocol class for a generic spindle controller.

    The controller inputs are passed to ctrl_step function, 
    this in turn computes the control signals that are properties of this class.
    """

    # Controller must write to these
    vfd_setpoint_hz:float 
    cvt_high_cmd:bool 
    cvt_low_cmd:bool 
    gearbox_backgear_active:bool 

    def ctrl_step(self,setpoint_rpm:float, spindle_act_rpm:float, cvt_at_max:bool, cvt_at_min:bool, vfd_at_speed:bool, dt:float): ...


class RotarySpindleSim:

    def __init__(self, conf:SimConfig, dt_s:float, total_sim_time:float):

        self.conf = conf 
        self.Tf = total_sim_time
        self.dt = dt_s

        # VFD rate limit
        self.vfd_rate_lim_hz = (conf.vfd_max_hz - conf.vfd_min_hz)/conf.vfd_ramp_time_s

        # cvt params
        self.a_0 = conf.k_cvt_low
        self.a_2 = conf.k_cvt_high - conf.k_cvt_low
        self.v_c_max = 1/5
        self.p_c_tol = self.v_c_max * self.conf.cvt_speed_tau_s

        self._init_vars()

    def _init_vars(self):
        # Logs signals during a simulation
        self.t = [0.0]

        self.vfd_f_cmd = []
        self.vfd_f_act = [0.0]
        self.vfd_n_act = [0.0]

        self.cvt_cmd = []
        self.p_cf =[0.0]
        self.p_c =[0.0]
        self.k_c = []

        self.k_tot = []

        self.sp_act = []

        self.k_gearbox = [1.0]


    def sim(self, ctrl:Controller):

        self._init_vars()

        N = int(self.Tf // self.dt)

        for i in range(N):
            
            setpoint = 0.0
            if self.dt*i > 0.5:
                setpoint = 2000

            k_cvt = self.a_0 + self.a_2*self.p_cf[-1]**2

            # Total gain (VFD RPM -> SP RPM)
            k_tot = self.k_gearbox[-1]*k_cvt

            # act rpm
            sp_act = self.vfd_n_act[-1]*k_tot

            # Controller
            cvt_at_max = self.p_cf[-1] >= 1 - self.p_c_tol
            cvt_at_min = self.p_cf[-1] <= self.p_c_tol

            vfd_at_speed = abs(ctrl.vfd_setpoint_hz - self.vfd_f_act[-1]) < 0.001

            ctrl.ctrl_step(setpoint_rpm=setpoint,spindle_act_rpm=sp_act, cvt_at_max=cvt_at_max, cvt_at_min=cvt_at_min, vfd_at_speed=vfd_at_speed, dt=self.dt)

            # VFD 

            # rate limits
            f_UL = self.vfd_f_act[-1] + self.vfd_rate_lim_hz*self.dt
            f_LL = self.vfd_f_act[-1] - self.vfd_rate_lim_hz*self.dt

            # VFD Frequency
            f_plus = np.clip(ctrl.vfd_setpoint_hz, f_LL, f_UL)

            # Velocity setpoint
            n_setp = self.vfd_f_act[-1]*self.conf.hz_2_rps*60
            #n_setp = f_plus*self.conf.hz_2_rps*60

            # Actual VFD rpm
            n_plus = self.vfd_n_act[-1] + (n_setp - self.vfd_n_act[-1])/self.conf.vfd_tau_s * self.dt

            # p_c - fwd euler
            u_c = 0.0
            if ctrl.cvt_high_cmd and not ctrl.cvt_low_cmd:
                u_c = 1.0 
            if not ctrl.cvt_high_cmd and ctrl.cvt_low_cmd:
                u_c = -1.0 

            # Velocity of p_c
            p_c_plus = self.p_c[-1] + self.v_c_max*u_c* self.dt 
            p_c_plus = np.clip(p_c_plus,0.0,1.0)

            p_cf_plus = self.p_cf[-1] + (p_c_plus - self.p_cf[-1])/self.conf.cvt_speed_tau_s * self.dt


            # Fixed gearbox
            k_gearbox_plus = 1.0
            if ctrl.gearbox_backgear_active:
                k_gearbox_plus = 1/6.5


            self.k_c.append(k_cvt)
            self.k_tot.append(k_tot)
            self.cvt_cmd.append(u_c)
            self.vfd_f_cmd.append(ctrl.vfd_setpoint_hz)
            self.sp_act.append(sp_act)

            if i < N - 1:
                
                self.t.append((i+1)*self.dt)
                self.vfd_f_act.append(f_plus)
                self.vfd_n_act.append(n_plus)

                self.p_cf.append(p_cf_plus)
                self.p_c.append(p_c_plus)
                self.k_gearbox.append(k_gearbox_plus)



    def plot_results(self,prefix = '',save_figs=False):



        fig, ax1 = plt.subplots()

        ax1.plot(self.t, self.vfd_f_cmd, label='VFD freq cmd')
        ax1.plot(self.t, self.vfd_f_act, label='VFD freq act')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Frequency [Hz]')
        ax1.grid(True)

        ax2 = ax1.twinx()
        ax2.plot(self.t, self.vfd_n_act, label='Mtr rpm act', color='green')
        ax2.set_ylabel('RPM')

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2)

        plt.title('VFD setpoint and actual response')
        plt.tight_layout()
        if save_figs:
            plt.savefig(f'docs/{prefix}vfd.png')

        plt.figure()
        plt.subplot(2,1,1)
        plt.plot(self.t, self.p_cf, label='CVT position p_c')
        #plt.plot(self.t, self.v_c, label='CVT velocity v_c')
        plt.plot(self.t, self.k_c, label='CVT ratio k_c')
        plt.xlabel('Time [s]')
        plt.ylabel('CVT variables')
        plt.title('CVT state and ratio')
        plt.legend()
        plt.grid(True)

        plt.subplot(2,1,2)
        plt.ylabel('CVT Command')
        plt.step(self.t, self.cvt_cmd, where='post', label='CVT command')

        plt.xlabel('Time [s]')
        plt.grid(True)
        plt.tight_layout()
        if save_figs:
            plt.savefig(f'docs/{prefix}cvt.png')

        fig, ax1 = plt.subplots()

        mtr_n_setp = np.array(self.vfd_f_cmd) * self.conf.hz_2_rps*60

        ax1.plot(self.t, mtr_n_setp, label='Setpoint Motor RPM')
        ax1.plot(self.t, self.vfd_n_act, label='Actual Motor RPM')
        
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Motor RPM')
        ax1.grid(True)

        ax2 = ax1.twinx()
        ax2.plot(self.t, self.sp_act, label='Spindle RPM',color='green')
        ax2.set_ylabel('Spindle RPM')

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        #ax1.legend(lines1 + lines2, labels1 + labels2,location='se')
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='lower right')

        plt.title('Motor and spindle RPM')
        plt.tight_layout()
        if save_figs:
            plt.savefig(f'docs/{prefix}mtr_spindle.png')



def get_default_sim_conf():

    return SimConfig(vfd_max_hz=70,
                     vfd_min_hz=10,
                     hz_2_rps=4,
                     vfd_ramp_time_s=10,vfd_tau_s=0.2,
                     k_cvt_high=0.25, k_cvt_low=0.0975, cvt_speed_tau_s=0.2, 
                     k_gear_high=1.0, k_gear_low=1/6.5)




class Basic(Controller):


    def __init__(self):

        self.t = 0.0

        self.vfd_setpoint_hz: float = 0.0
        self.cvt_high_cmd: bool = False
        self.cvt_low_cmd: bool = False
        self.gearbox_backgear_active: bool = False
    
    def ctrl_step(self, setpoint_rpm:float, spindle_act_rpm: float, cvt_at_max: bool, cvt_at_min: bool, vfd_at_speed:bool, dt:float):


        self.cvt_high_cmd = False
        self.cvt_low_cmd = False

        if self.t < 0.5:
            pass 
        elif self.t < 5:
            self.vfd_setpoint_hz = 10
        elif self.t < 20:
            self.vfd_setpoint_hz = 70
        elif self.t < 30:
            self.vfd_setpoint_hz = 50
        elif self.t < 37:
            self.cvt_high_cmd = True 
        elif self.t < 60:
            self.cvt_low_cmd = True 
        
        self.t += dt


def main():

    conf = get_default_sim_conf()
    
    sim = RotarySpindleSim(conf,0.01,55)

    ctrl = Basic()

    sim.sim(ctrl)

    sim.plot_results(prefix='OL_',save_figs=True)

    plt.show()

if __name__ == '__main__':
    main()

