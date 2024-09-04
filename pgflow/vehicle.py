from __future__ import annotations
import numpy as np
import math

# from typing import List
from numpy.typing import ArrayLike

from pgflow.panel_flow import PanelFlow
from pgflow.arena import ArenaMap
from pgflow.building import Building
from pgflow.PIDcontroller import VehicleDynamics, PIDController
from pgflow.dynamics import dynamics_matrices, get_next_X
"""##Vehicles"""

from dataclasses import dataclass, field

from solver.a_star_maze import Maze as asMaze
# from pgflow.a_star_maze import Maze as asMaze


@dataclass(slots=True)
class PersonalVehicle:
    """Vehicle with the minimal required information: position, velocity, altitude, state etc"""

    ID: str
    position: ArrayLike
    goal: ArrayLike
    # checkpoints_list:ArrayLike
    tether_mode = None
    source_strength: float
    sink_strength: float = 5
    imag_source_strength: float = 0.75
    # state is unnused in PersonalVehicle FIXME for now
    state: int = 0


class Vehicle:
    _id_counter = 0
#Yuan_edit_below: 
    def __init__(self, source_strength: float = 0, imag_source_strength: float = 0.75, obstacles = None, graph = None, concave_record = None, theory_map = None, maze_width = None, maze_height = None, tether_mode = 0):
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        self.ID = f"V{Vehicle._id_counter}"
        Vehicle._id_counter += 1
        self._position = np.zeros(3)
        self.goal = np.zeros(3)

#Yuan_edit_below:
        # self._last_position = np.zeros(3)
        # self._farest_distance
        self.poschange_switch = 1
        self.last_pos = self._position
        self.last_is_further = 0
        self.is_further = 0  #0 means getting near, 1 means getting far
        self.dist_detector = 0 #0-39 means shut down, >=40 means start up


        self.cp_generate_counter = 0
        self.last_finalgoal = None
        self.maze_width = maze_width
        self.maze_height = maze_height
        self.checkpoints_list = []
        self.tether_mode = tether_mode
        if self.tether_mode == 1:
            self.obstacles = obstacles
            self.graph = graph
            self.concave_record = concave_record
            self.theory_map = theory_map
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        self.source_strength: float = source_strength
        self.sink_strength: float = 0
        self.imag_source_strength: float = imag_source_strength

        self.max_avoidance_distance: float = 20
        self.panel_flow = PanelFlow(self)
        Kp, Ki, Kd = 40, 0.1, 30
        self.dynamics = VehicleDynamics(mass=1, min_accel=-2, max_accel=2)
        self.pid_x = PIDController(
            Kp, Ki, Kd, self.dynamics.min_accel, self.dynamics.max_accel
        )
        self.pid_y = PIDController(
            Kp, Ki, Kd, self.dynamics.min_accel, self.dynamics.max_accel
        )
        self.desired_vectors = []
        self.pid_output = []

        self.arena: ArenaMap = None

        # self.desiredpos = np.zeros(3)
        self.velocity = np.zeros(3)
        self.path = np.zeros((1, 3))
        # FIXME there is a magic number of 0.2 for the destination, fix this
        self.state = 0
        # originally 1/50
        self._delta_t = 1 / 50  # 1/300 or less for vortex method, 1/50 for hybrid
        self.personal_vehicle_dict: dict[str, PersonalVehicle] = []
        self.relevant_obstacles: list[Building] = []
        self.transmitting = True
        self.max_speed = 0.8
        self.ARRIVAL_DISTANCE = 0.8

        self.has_landed = False
        self.turn_radius: float = 0.1  # max turn radius in meters
        self.v_free_stream: np.ndarray = np.zeros(
            2
        )  # velocity constantly pushing the vehicle towards its goal
        self.v_free_stream_mag = 0.5
        self.matA, self.matB = dynamics_matrices(self.delta_t)

    @property
    def position(self): 
        return self._position

    @position.setter
    def position(self, new_position):
        self._position = new_position
        vector_to_goal = (self.goal[:2] - new_position[:2])[:2]
        if np.all(vector_to_goal == 0):
            # non zero vector of ones in case the vehicle is exactly on the goal
            self.v_free_stream = np.ones(2)
        else:
            self.v_free_stream = (
                self.v_free_stream_mag * vector_to_goal / np.linalg.norm(vector_to_goal)
            )

    @property
    def delta_t(self):
        return self._delta_t

    @delta_t.setter
    def delta_t(self, new_delta_t):
        self._delta_t = new_delta_t
        self.matA, self.matB = dynamics_matrices(new_delta_t)

    def update_personal_vehicle_dict(
        self, case_vehicle_list: list[Vehicle], max_avoidance_distance: float = 100.0
    ) -> None:
        for v in case_vehicle_list:
            # overall options are: keep my previous knowledge, update it, or remove the drone entirely
            if v.ID == self.ID:
                # we should not be in our own list, so remove us if we are
                # TODO in future we should never be added in the first place so FIXME
                

#Yuan_edit_below: 
                # if self.tether_mode == 1 and len(self.checkpoints_list) != 0:
                #     continue
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


                self.personal_vehicle_dict.pop(self.ID, None)
                continue

            if v.transmitting == True:
                # other vehicle is transmitting so either take the newer vehicle info or remove it entirely if too far or arrived
                if v.state == 1:
                    # arrived, so remove from list, we don't care about it
#Yuan_edit_below: 
                    # if self.tether_mode == 1 and len(self.checkpoints_list) != 0:
                    #     continue
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    self.personal_vehicle_dict.pop(v.ID, None)

                else:
                    # not arrived, check if close enough
                    if (
                        np.linalg.norm(v.position - self.position)
                        > max_avoidance_distance
                    ):
                        # too far, remove
                        self.personal_vehicle_dict.pop(v.ID, None)
                    else:
                        # not too far, update or add
                        self.personal_vehicle_dict[v.ID] = PersonalVehicle(
                            **v.basic_properties()
                        )
            else:
                # not transmitting, keep the old, aka do nothing
                pass
        return None

    def update_nearby_buildings(self, threshold: float = 5) -> None:
        # position = Point(self.position)
        self.relevant_obstacles = self.arena.get_nearby_buildings(
            self.position, threshold
        ) 
        for building in self.relevant_obstacles:
            if building.K_inv is None:

                building.panelize(self.arena.size)
                building.calculate_coef_matrix()
        # print(self.relevant_obstacles)
        return None

    def basic_properties(self):
        return {
            "ID": self.ID,
            "position": self.position,
            "goal": self.goal,
            "source_strength": self.source_strength,
            "sink_strength": self.sink_strength,
        }

    def set_initial_position(self, pos):
        self.position = np.array(pos) 
        self.path = np.array(pos)
        self.path = self.path.reshape(1, 3)
        #NOTE, removed the lines below as immediately setting the state to 1 is problematic
        # if self.arrived(self.ARRIVAL_DISTANCE):
            # if self.checkpoints_list:
            #     self.Set_Goal(self.checkpoints_list.pop(0))
            #     print("Vehicle",self.ID,":temporary goal is reached")
            # else:
            #     print("Vehicle",self.ID,"final goal is reached")
            #     # goal has been reached
            #     self.state = 1

    def Set_Goal(self, goal, goal_strength, cp_generator = 1):
        new_goal = np.array(goal)
        new_sink_strength = goal_strength
        if cp_generator == 1 and self.tether_mode == 1:
            if self.last_finalgoal is None:

                self.goal = new_goal
                # print("\nVehicle",self.ID, ":final goal setted as :", self.goal,"\n")
                self.sink_strength = new_sink_strength
                self.last_finalgoal = self.goal
                # print("Vehicle",self.ID, ":recursion start !","\n")
                self.recur_Set_Goal(self.sink_strength)
            else:
                # print("curGoal is:",self.goal,"\nlastGoal is:", self.last_finalgoal)
                dist = math.sqrt((new_goal[0] - self.last_finalgoal[0])**2 + (new_goal[1] - self.last_finalgoal[1])**2)
                # print("cp_generator:",cp_generator,"\ndist:",dist)
                if dist > self.ARRIVAL_DISTANCE:
                    # print("new_goal =",new_goal)
                    self.goal = new_goal
                    self.sink_strength = new_sink_strength
                    self.last_finalgoal = self.goal
                    # print("Vehicle",self.ID, ":recursion start again !","\n")
                    self.recur_Set_Goal(self.sink_strength)
                    # print("Vehicle",self.ID, ":next temporary goal is", self.goal,"\n")

        else:
            self.goal = new_goal
            self.sink_strength = new_sink_strength
        # elif len(self.checkpoints_list) == 0:
        #     print("Vehicle",self.ID, ":final goal is", self.goal)
        #     # print("checkpoints list remains:", self.checkpoints_list)
        # else:
        #     print("Vehicle",self.ID, ":next temporary goal is", self.goal,"\n")
            # print("checkpoints list remains:", self.checkpoints_list)

        # print("Vehicle",self.ID, ":final goal is", self.goal)

            
    
    def recur_Set_Goal(self, goal_strength):
        # self.cp_generate_counter += 1
        # print("cp_generate_counter :", self.cp_generate_counter)
        self.get_checkpoints_list()
        if self.checkpoints_list is None:
            print("checkpoints generateing failed")

        # print("final goal of this recur:",self.goal,"\n")
        goalCur = self.checkpoints_list.pop(0)
        if len(goalCur) == 2:
            goalCur.append(1.2)
        self.Set_Goal(goalCur, goal_strength, 0)

    def run_flow_vel_calc_only(self):
        flow_vels = self.panel_flow.Flow_Velocity_Calculation(self)
        mag = np.linalg.norm(flow_vels)
        # induced velocity unit vector
        unit_new_velocity = flow_vels / mag
        return unit_new_velocity

    def run_simple_sim(self, mode: str):
        """run one iteration of the simulation
        mode:str (standard | radius | pid)"""
        # these are the flow velocities induced on every vehicle (based off the personal vehicle list), stored as a list
        flow_vels = self.panel_flow.Flow_Velocity_Calculation(self)

        mag = np.linalg.norm(flow_vels)
        # induced velocity unit vector
        unit_new_velocity = flow_vels / mag
        self.desired_vectors.append(unit_new_velocity.tolist())
        
        if mode == "radius":
            self.update_position_max_radius(flow_vels)
        elif mode == "pid":
            self.update_position_pid(flow_vels)
        elif mode == "dynamics":
            self.update_position_dynamics(flow_vels)
        elif mode == "fancy":
            self.update_position_dynamics_fancy(flow_vels)
        else:
            self.update_position_clipped(flow_vels)

    def update_position_dynamics_fancy(self, flow_vel):
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me"""
        current_position = self.position.copy()
        # temporarily set my position a bit in the future
        X = np.array(
            [*self.position, *self.velocity]
        )  # Example values for current state
        V_des = np.append(flow_vel, 0)*self.max_speed
        self.position = get_next_X(X, V_des, delta_t=2, num_points=5)[
            :3
        ]  # position 0.5 seconds in the future 
        future_flow_vel = self.panel_flow.Flow_Velocity_Calculation(self)
        self.position = current_position  # reset to current position 
        flow_vel = flow_vel + 0.7 * future_flow_vel
        V_des = np.append(flow_vel, 0)

        # TODO currently array is 2D, so add a third dimension for compatibility
        #########################################
        # magnitude of the induced velocity vector
        mag = np.linalg.norm(V_des)
        # # induced velocity unit vector
        V_des_unit = V_des / mag
        # self.desired_vectors.append(V_des_unit[:2].tolist())

        # Define your current state X and control input U
        X = np.array(
            [*self.position, *self.velocity]
        )  # Example values for current state
        U = V_des_unit * self.max_speed  # Control inputs based on desired direction
        # Calculate the next state directly
        X_next = self.matA @ X + self.matB @ U

        # position and velocity updated
        # self.last_pos = self.position
        self.position, self.velocity = X_next[:3], X_next[3:6] 


#pos_update_____________
        # if self.poschange_switch == 1:
        #     self.update_is_further()
        #     if self.dist_detector < 10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #             print("detector:",self.dist_detector)
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >= 10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________        self.path = np.vstack((self.path, self.position))

        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):
            if self.checkpoints_list:
                self.Set_Goal(self.checkpoints_list.pop(0),self.sink_strength, 0)
                # print("Vehicle",self.ID,":temporary goal is reached")
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1 
                print("final goal is reached")
                
        return self.position

    def update_position_dynamics(self, flow_vel):
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me"""
        # TODO currently array is 2D, so add a third dimension for compatibility
        V_des = np.append(flow_vel, 0)
        #########################################
        # magnitude of the induced velocity vector
        mag = np.linalg.norm(V_des)
        # induced velocity unit vector
        # if mag == 0 or np.isnan(mag):
        V_des_unit = V_des / mag

        # self.desired_vectors.append(V_des_unit[:2].tolist())

        # Define your current state X and control input U
        X = np.array(
            [*self.position, *self.velocity]
        )  # Example values for current state
        U = V_des_unit * self.max_speed  # Control inputs based on desired direction
        # Calculate the next state directly
        X_next = self.matA @ X + self.matB @ U

        # self.last_pos = self.position
        self.position, self.velocity = X_next[:3], X_next[3:6] 

        self.path = np.vstack((self.path, self.position))
        
#pos_update_____________
        # if self.poschange_switch == 1: 
        #     self.update_is_further()
        #     if self.dist_detector <  10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >=  10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________ 

        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):
            if self.checkpoints_list:
                self.Set_Goal(self.checkpoints_list.pop(0), self.sink_strength, 0)
                # print("Vehicle",self.ID,":temporary goal is reached")
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1 
                print("final goal is reached")
                
        return self.position

    def update_position(self, flow_vel):
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me, self.velocity is used, so that the movement is less brutal"""

        # K is vehicle speed coefficient, a design parameter
        # flow_vels = flow_vels * self.delta_t
        V_des = flow_vel
        # magnitude of the induced velocity vector
        mag = np.linalg.norm(V_des)
        # induced velocity unit vector

        V_des_unit = V_des / mag
        # set z component to 0
        V_des_unit[2] = 0
        # force mag to lie between 0 and 1
        mag_clipped = np.clip(mag, 0.0, self.max_speed)  # 0.3 tello 0.5 pprz
        # set the magnitude of the induced velocity vector to mag_converted (ie the clipped value between 0 and 1)
        clipped_velocity = V_des_unit * mag_clipped
        self.velocity += clipped_velocity
        self.velocity = self.velocity / np.linalg.norm(self.velocity)

        # multiply the flow velocity by some predefined constant, this is sort of like imposing the delaT
        # change in position = ds = v dt so velocitygain is actually dt here
        delta_s = self.velocity * self.delta_t

        # self.last_pos = self.position
        if self.tether_mode == 1:
            self.position[:2] = self.position[:2] + np.array(delta_s)[:2]
        else: 
            self.position = self.position + np.array(delta_s) 

        self.path = np.vstack((self.path, self.position))
        # print("position update mode: update_position:")
        
#pos_update_____________
        # if self.poschange_switch == 1: 
        #     self.update_is_further()
        #     if self.dist_detector <  10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >=  10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________ 
        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):  # 0.1 for 2d
            if self.checkpoints_list:
                self.Set_Goal(self.checkpoints_list.pop(0), self.sink_strength, 0)
                # print("Vehicle",self.ID,":temporary goal is reached")
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1 
                print("final goal is reached")
        return self.position

    def update_position_clipped(self, flow_vel):
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me"""

        # TODO currently array is 2D, so add a third dimension for compatibility
        V_des = np.append(flow_vel, 0)
        #########################################
        # magnitude of the induced velocity vector
        mag = np.linalg.norm(V_des)
        # induced velocity unit vector
        V_des_unit = V_des / mag
        # self.desired_vectors.append(V_des_unit[:2].tolist())

        self.velocity = V_des_unit * self.max_speed
        delta_s = self.velocity * self.delta_t  # use unit velocity
        # self.last_pos = self.position
        self.position = self.position + np.array(delta_s)
        self.path = np.vstack((self.path, self.position))
        
#pos_update_____________
        # if self.poschange_switch == 1: 
        #     self.update_is_further()
        #     if self.dist_detector <  10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >=  10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________ 
        
#Yuan_edit_below: 
        # print("position update mode: clipped")
        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):
            # print("\nclipped: checkpoints list remains:", self.checkpoints_list)
            # print("\nlen(self.checkpoints_list)", len(self.checkpoints_list))
            if len(self.checkpoints_list) != 0:
                # print("Vehicle",self.ID,":temporary goal is reached")
                self.Set_Goal(self.checkpoints_list.pop(0), self.sink_strength, 0)
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1 
                print("final goal is reached")
                
        return self.position
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    def update_position_max_radius(self, flow_vel: np.ndarray):
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me"""
        # magnitude of the induced velocity vector in 2D
        mag = np.linalg.norm(flow_vel)
        # induced velocity unit vector
        unit_new_velocity = flow_vel / mag
        # self.desired_vectors.append(unit_new_velocity.tolist())

        speed = np.linalg.norm(self.velocity[:2])
        if speed == 0:
            # initial velocity is just new velocity at start of simulation
            unit_old_velocity = unit_new_velocity
        else:
            unit_old_velocity = self.velocity[:2] / speed

        if not self.turn_radius < self.max_speed * self.delta_t / 2:

            # this can be below -1 if the radius is less than vdt/2 which makes the turn radius mathematically unachievable
            min_cos = 1 - 0.5 * (speed * self.delta_t / self.turn_radius) ** 2

            # Calculate the angle in radians, and then convert it to degrees
            # The np.clip is used to handle potential floating-point arithmetic issues that might push the dot product
            # slightly outside the range [-1, 1], which would cause np.arccos to return NaN
            cos_angle = np.clip(np.dot(unit_new_velocity, unit_old_velocity), -1.0, 1.0)

            if cos_angle < min_cos:
                max_theta = np.arccos(np.clip(min_cos, -1.0, 1.0))
                # in numpy cross product of two 2d vectors returns the z component of the resulting vector
                cross_product = np.cross(unit_new_velocity, unit_old_velocity)

                if cross_product > 0:
                    # clockwise
                    theta = -max_theta
                else:
                    # anti_clockwise
                    theta = max_theta

                # Create the rotation matrix
                rotation_matrix = np.array(
                    [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
                )

                # Rotate the vector
                unit_new_velocity = np.dot(rotation_matrix, unit_old_velocity)

        unit_new_velocity = np.append(unit_new_velocity, 0)

        # multiply the flow velocity by some predefined constant to set max speed
        self.velocity = unit_new_velocity * self.max_speed
        delta_s = self.velocity * self.delta_t  # use unit velocity

        # self.last_pos = self.position
        if self.tether_mode == 1:
            self.position[:2] = self.position[:2] + np.array(delta_s)[:2]
        else: 
            self.position = self.position + np.array(delta_s) 

        self.path = np.vstack((self.path, self.position))
        
#pos_update_____________
        # if self.poschange_switch == 1: 
        #     self.update_is_further()
        #     if self.dist_detector <  10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >=  10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________ 

 #Yuan_edit_below: 
        # print("position update mode: max_radius:")
        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):
            if self.checkpoints_list:
                self.Set_Goal(self.checkpoints_list.pop(0), self.sink_strength, 0)
                # print("Vehicle",self.ID,":temporary goal is reached")
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1 
                print("final goal is reached")
                
        return self.position
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    def update_position_pid(self, flow_vel: ArrayLike) -> None:
        """Updates my position within the global vehicle_list given the induced flow from other vehicles onto me"""
        # magnitude of the induced velocity vector in 2D
        mag = np.linalg.norm(flow_vel)
        # induced velocity unit vector
        unit_new_velocity = flow_vel / mag
        # self.desired_vectors.append(unit_new_velocity.tolist())
        # define desired position
        desired_position = self.position[:2] + unit_new_velocity * self.delta_t * 10

        x_desired, y_desired = desired_position[0], desired_position[1]
        # define current position and velocity
        x, y = self.position[0], self.position[1]
        vx, vy = self.velocity[0], self.velocity[1]

        # obtain error
        error_x = x_desired - x
        error_y = y_desired - y

        # obtain desired accelerations from pid controllers
        ax = self.pid_x.update(error_x, self.delta_t)
        ay = self.pid_y.update(error_y, self.delta_t)

        # define velocity limits
        min_velocity, max_velocity = (-1,1)

        # Adjust acceleration based on current velocity and limits
        ax = self.pid_x.adjust_acceleration(
            vx, ax, max_velocity, min_velocity, self.delta_t
        )
        ay = self.pid_y.adjust_acceleration(
            vy, ay, max_velocity, min_velocity, self.delta_t
        )

        self.pid_output.append([ax, ay])
        x, vx, y, vy = self.dynamics.update_state_verlet(
            x, vx, y, vy, ax, ay, self.delta_t
        )
        # set vehicle state to new state

        # self.last_pos = self.position
        self.position = np.array([x, y, self.position[2]])
        self.velocity[0], self.velocity[1] = vx, vy 

        self.path = np.vstack((self.path, self.position))
        
#pos_update_____________
        # if self.poschange_switch == 1: 
        #     self.update_is_further()
        #     if self.dist_detector <  10:
        #         if self.is_further == 1:
        #             self.dist_detector += 1
        #         else:
        #             self.dist_detector = 0
        #     elif self.dist_detector >=  10:
        #         print("detector >=  10")
        #         if self.is_further == 0:
        #             print("regenerate cpList")
        #             self.last_finalgoal = None
        #             self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
        #             self.dist_detector = 0
#pos_update_____________ 

#Yuan_edit_below: 
        # print("position update mode: pid:")
        if self.arrived(arrival_distance=self.ARRIVAL_DISTANCE):
            if self.checkpoints_list:
                self.Set_Goal(self.checkpoints_list.pop(0), self.sink_strength, 0)
                # print("Vehicle",self.ID,":temporary goal is reached")
            else:
                # print("Vehicle",self.ID,"final goal is reached")
                # goal has been reached
                self.state = 1
                print("final goal is reached")
                
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



    def arrived(self, arrival_distance):
        """Similar to state but using the current live position"""
#Yuan_edit_below: 
        if self.tether_mode == 1 :
            distanceCur = np.linalg.norm(self.goal[:2] - self.position[:2]) 
        else:
            distanceCur = np.linalg.norm(self.goal - self.position) 
        # print("currently position:",self.position, "\ngoal:",self.goal)
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        arrived = distanceCur < arrival_distance
        # print("distanceCur =", distanceCur,"\narrival_distance =", arrival_distance)
        # print("detecting:", self.position)
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        return arrived 

#Yuan_edit_below: 
    def get_checkpoints_list(self, r = None): 
        mazeP = None
        mazeP = asMaze(self.obstacles,list(self.goal), self.maze_width, self.maze_height, self.concave_record, self.graph, self.theory_map)
        mazeP.A_star_path(tuple(self.position))
        if r == None:
            mazeP.generate_cpList(0.4)
        else:
            mazeP.generate_cpList(r)
        # print("Vehicle- ",self.ID,"'s checkpoints_list:",mazeP.sPath,"\n")
        self.checkpoints_list = [list(checkpoints) for checkpoints in mazeP.sPath[1:]]

    def squared_distance(self, p1, p2):
        return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2

    def update_is_further(self):
        cur_dist = self.squared_distance(self.position, self.goal)
        last_dist = self.squared_distance(self.last_pos, self.goal)
        if cur_dist < last_dist:
            self.is_further = 0
        else:
            self.is_further = 1

            
#pos_update_____________

    def setter_position(self, pos, poschange_switch = None):
        self.last_pos = self.position
        self.position = pos
        if poschange_switch is not None:
            self.poschange_switch = poschange_switch

        if self.poschange_switch == 1: 
            self.update_is_further()
            if self.dist_detector <  10:
                if self.is_further == 1:
                    self.dist_detector += 1
                else:
                    self.dist_detector = 0
            elif self.dist_detector >=  10:
                print("detector >=  10")
                if self.is_further == 0:
                    print("regenerate cpList")
                    self.last_finalgoal = None
                    self.Set_Goal(self.checkpoints_list[-1],self.sink_strength,1)
                    self.dist_detector = 0
#pos_update_____________ 
#Yuan_edit_above^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


