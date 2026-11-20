import numpy as np
from types import SimpleNamespace
from PrelaunchSolution import get_poca_stats 
from Paramaters import * 

aero = AeroParams()
sim = SimParams()
missile = MissileParams()
env = Enviroment()
launcher = LauncherParams()
ground = GroundParams()

class MissileSimulation:
    def __init__(self):
        self.aero = aero
        self.sim = sim
        self.missile = missile
        self.env = env
        self.launcher = launcher
        self.ground = ground



    def missile_propogator(self, launch_angle, pip):
        
        ## simulates the missile flyout to find the trajectory and relevant statistics based on the launch angle and target trajectory
        
        time = 0.0
        time_max = 200
        iter = 0
        pip_passed = False
            
    
        # initial state for vehicle off rail
            
        
        # initial gamma 
        gamma = launch_angle
        v = 30 # 
        
        missile_state = SimpleNamespace(x = self.launcher.launcher_emplacement_x,
                                            z = self.launcher.launcher_emplacement_z,
                                            vx = v * np.cos(gamma),
                                            vz = -v * np.sin(gamma),
                                            alpha = np.deg2rad(5),
                                            q = 0, 
                                            theta = gamma)
            
        
    
            
        state_log = [SimpleNamespace(**vars(missile_state))]
        time_log = [SimpleNamespace(time=time)]
     
            
            
        while missile_state.z <= 0.0 and time < time_max and pip_passed == False :
            
                time+= self.ground.extrap_time_step
                iter += 1
                
               

                
                #  Compute Aero Forces/Moments and Thrust 
                
                T = self.get_thrust(time)
                AeroForces = self.get_aero_forces(missile_state)
                
                D = AeroForces.Drag
                L = AeroForces.Lift
                M = AeroForces.Moment

              
                

              
                    gammadot =   (T*np.sin(missile_state.alpha) )/(self.missile.m*v) - self.env.gravity * np.cos(gamma) / v





                    ## future: add alpha and q dynamics with zero control effort (aka Zero Effort Trajectory)
                qdot = M/self.missile.Iy
                alphadot = missile_state.q - gammadot
                vdot = T*np.cos(missile_state.alpha)/self.missile.m -self.env.gravity * np.sin(gamma) - D/self.missile.m
            
        
                v = v + vdot * self.ground.extrap_time_step
                gamma = gamma + gammadot * self.ground.extrap_time_step
                missile_state.vx = v * np.cos(gamma)
                missile_state.vz = -v * np.sin(gamma)
                
                missile_state.alpha = missile_state.alpha + alphadot * self.ground.extrap_time_step
                missile_state.x = missile_state.x + missile_state.vx * self.ground.extrap_time_step
                missile_state.z = missile_state.z + missile_state.vz * self.ground.extrap_time_step
        
               
    
                
                if iter >= len(target_traj):
                    # target window ended before POCA / impact; stop at the last known target state
                    break
                
                target_state = target_traj[iter]
                
                state_log.append(SimpleNamespace(**vars(missile_state)))
                time_log.append(SimpleNamespace(time=time))
                
                # scalar engagement stats at this step, for the stopping condition
                
                prel = np.array([target_state.x, target_state.z]) - np.array([missile_state.x, missile_state.z])
                vrel = np.array([target_state.vx, target_state.vz]) - np.array([missile_state.vx, missile_state.vz])
                rng = np.linalg.norm(prel)
                range_rate = np.dot(prel, vrel) / max(rng, 1e-9)
                
                # gets poca stats 
                
                
                
                if range_rate >= 0:
                    poca_stats = get_poca_stats(state_log, target_traj, tgo_max)
                    break
                
            
        poca_stats = get_poca_stats(state_log, target_traj, tgo_max)        
       
               
            
                
        
        return state_log, time_log, poca_stats
        