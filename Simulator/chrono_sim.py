import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math
import sys
from Simulator.driver import Driver

# =============================================================================
# class Vehicle:
#
chrono.SetChronoDataPath('/home/aaron/chrono/data/')
veh.SetDataPath('/home/aaron/chrono/data/vehicle/')

def GetInitPose(p1, p2):
    initLoc = chrono.ChVectorD(p1[0], p1[1], 0.5)

    vec = chrono.ChVectorD(p2[0], p2[1], 0.5) - chrono.ChVectorD(p1[0], p1[1], 0.5)
    theta = math.acos((chrono.ChVectorD(1,0,0)^vec)/(vec.Length()))
    initRot = chrono.ChQuaternionD()
    initRot.Q_from_AngZ(theta)

    return initLoc, initRot

class ChronoSim:
    def __init__(self, step_size, track, irrlicht=False, initLoc=chrono.ChVectorD(0,0,0), initRot=chrono.ChQuaternionD(1,0,0,0)):
        # Vehicle parameters for matplotlib
        self.length = 4.5  # [m]
        self.width = 2.0  # [m]
        self.backtowheel = 1.0  # [m]
        self.wheel_len = 0.3  # [m]
        self.wheel_width = 0.2  # [m]
        self.tread = 0.7  # [m]
        self.wb = 2.5  # [m]

        # Chrono parameters
        self.step_size = step_size
        self.irrlicht = irrlicht
        self.step_number = 0

        # Time interval between two render frames
        self.render_step_size = 1.0 / 60  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))

        # JSON file for vehicle model
        self.vehicle_file = veh.GetDataPath() + "hmmwv/vehicle/HMMWV_Vehicle.json"

        # JSON files for terrain
        self.rigidterrain_file = veh.GetDataPath() + "terrain/RigidPlane.json"

        # JSON file for powertrain (simple)
        self.simplepowertrain_file = veh.GetDataPath(
        ) + "generic/powertrain/SimplePowertrain.json"

        # JSON files tire models (rigid)
        self.rigidtire_file = veh.GetDataPath() + "hmmwv/tire/HMMWV_RigidTire.json"

        # Initial vehicle position
        self.initLoc = initLoc

        # Initial vehicle orientation
        self.initRot = initRot

        # Rigid terrain dimensions
        self.terrainHeight = 0
        self.terrainLength = 300.0  # size in X direction
        self.terrainWidth = 300.0  # size in Y direction

        # Point on chassis tracked by the camera (Irrlicht only)
        self.trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

        # --------------------------
        # Create the various modules
        # --------------------------

        self.vehicle = veh.WheeledVehicle(
            self.vehicle_file, chrono.ChMaterialSurface.NSC)
        self.vehicle.Initialize(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetStepsize(self.step_size)
        self.vehicle.SetChassisVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

        # Create and initialize the powertrain system
        self.powertrain = veh.SimplePowertrain(self.simplepowertrain_file)
        self.vehicle.InitializePowertrain(self.powertrain)

        # Create and initialize the tires
        for axle in self.vehicle.GetAxles():
            tireL = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
            tireR = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

        # Create the ground
        self.terrain = veh.RigidTerrain(
            self.vehicle.GetSystem(), self.rigidterrain_file)

        # -------------
        # Create driver
        # -------------
        self.driver = Driver(self.vehicle)

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        self.steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
        self.throttle_time = 1.0  # time to go from 0 to +1
        self.braking_time = 0.3   # time to go from 0 to +1
        self.driver.SetSteeringDelta(
            self.render_step_size / self.steering_time)
        self.driver.SetThrottleDelta(
            self.render_step_size / self.throttle_time)
        self.driver.SetBrakingDelta(self.render_step_size / self.braking_time)

        if self.irrlicht:
            self.DrawTrack(track)

        if self.irrlicht:
            self.app = veh.ChVehicleIrrApp(self.vehicle)
            self.app.SetHUDLocation(500, 20)
            self.app.SetSkyBox()
            self.app.AddTypicalLogo()
            self.app.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100,
                                      100)
            self.app.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100,
                                      100)
            self.app.EnableGrid(False)
            self.app.SetChaseCamera(self.trackPoint, 6.0, 0.5)

            self.app.SetTimestep(self.step_size)
            # ---------------------------------------------------------------------
            #
            #  Create an Irrlicht application to visualize the system
            #
            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
            # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
            # If you need a finer control on which item really needs a visualization proxy
            # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

            self.app.AssetBindAll()

            # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
            # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

            self.app.AssetUpdateAll()

    def DrawTrack(self, track, z=0.5):
        if self.irrlicht:
            road = self.vehicle.GetSystem().NewBody()
            road.SetBodyFixed(True)
            self.vehicle.GetSystem().AddBody(road)

            def toChPath(path):
                ch_path = chrono.vector_ChVectorD()
                for x,y in zip(path.x, path.y):
                    point = chrono.ChVectorD(x, y, z)
                    ch_path.push_back(point)
                return chrono.ChBezierCurve(ch_path)

            num_points = len(track.center.x)
            path_asset = chrono.ChLineShape()
            path_asset.SetLineGeometry(
                chrono.ChLineBezier(toChPath(track.center)))
            path_asset.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
            path_asset.SetNumRenderPoints(max(2 * num_points, 400))
            road.AddAsset(path_asset)

    def Advance(self, step):
        if self.irrlicht:
            if not self.app.GetDevice().run():
                return -1
            if self.step_number % self.render_steps == 0:
                self.app.BeginScene(
                    True, True, chronoirr.SColor(255, 140, 161, 192))
                self.app.DrawAll()
                self.app.EndScene()
        else:
            self.vehicle.GetSystem().DoStepDynamics(step)

        # Collect output data from modules (for inter-module communication)
        driver_inputs = self.driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = self.vehicle.GetSystem().GetChTime()

        self.driver.Synchronize(time)
        self.vehicle.Synchronize(time, driver_inputs, self.terrain)
        self.terrain.Synchronize(time)
        if self.irrlicht:
            self.app.Synchronize("", driver_inputs)

        # Advance simulation for one timestep for all modules
        self.driver.Advance(step)
        self.vehicle.Advance(step)
        self.terrain.Advance(step)
        if self.irrlicht:
            self.app.Advance(step)
            self.step_number += 1

    def Close(self):
        if self.app.GetDevice().run():
            self.app.GetDevice().closeDevice()

    class State:
        """
        vehicle state class
        """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v

    def GetState(self):
        """
        Returns State:
            [x,y,v,heading]
        """
        return self.State(
            x=self.vehicle.GetVehiclePos().x,
            y=self.vehicle.GetVehiclePos().y,
            yaw=self.vehicle.GetVehicleRot().Q_to_Euler123().z,
            v=self.vehicle.GetVehicleSpeed())


class GenericCar():
    """
    generic car class
    """
    class State:
        """
        vehicle state class
        """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w_e=0.0, w_e_dot=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.w_e = w_e
            self.w_e_dot = w_e_dot
            self.predelta = None

        def __getitem__(self, key):
            return self[key]

        def __str__(self):
            return str('({}, {}, {}, {})'.format(self.x, self.y, self.yaw, self.v))

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.max_steer = np.deg2rad(45.0)  # maximum steering angle [rad]
        self.max_dsteer = np.deg2rad(30.0)  # maximum steering speed [rad/s]
        self.max_speed = 55.0 / 3.6  # maximum speed [m/s]
        self.min_speed = -20.0 / 3.6  # minimum speed [m/s]
        self.max_accel = 1.0  # maximum accel [m/ss]

        # Vehicle parameters
        self.length = 4.5  # [m]
        self.width = 2.0  # [m]
        self.backtowheel = 1.0  # [m]
        self.wheel_len = 0.3  # [m]
        self.wheel_width = 0.2  # [m]
        self.tread = 0.7  # [m]
        self.wb = 2.5  # [m]
        self.mass = 1500  # [kg]

        # Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81

        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        # Tire force
        self.c = 10000
        self.F_max = 10000

        self.state = self.State(x, y, yaw, v)

        self.steer = 0
        self.throttle = 0
        self.braking = 0

    def Update(self, throttle, steering, braking, step):
        steering = np.deg2rad(steering)

        # input check
        if steering >= self.max_steer:
            steering = self.max_steer
        elif steering <= -self.max_steer:
            steering = -self.max_steer

        F_aero = self.c_a * self.state.v ** 2
        R_x = self.c_r1 * self.state.v
        F_g = self.mass * self.g * np.sin(steering)
        if braking >= 0:
            braking = 1
        F_load = (F_aero + R_x + F_g) * braking
        T_e = throttle * (self.a_0 + self.a_1 * self.state.w_e +
                          self.a_2 * self.state.w_e * self.state.w_e)
        W_w = self.GR * self.state.w_e
        if W_w == 0:
            r_eff = 0
        else:
            r_eff = self.state.v / W_w
        if self.state.v == 0:
            s = 0
        else:
            s = (W_w * self.r_e - self.state.v) / self.state.v
        cs = self.c * s

        if abs(s) < 1:
            F_x = cs
        else:
            F_x = self.F_max

        a = (F_x - F_load) / self.mass

        self.state.x = self.state.x + self.state.v * \
            math.cos(self.state.yaw) * step
        self.state.y = self.state.y + self.state.v * \
            math.sin(self.state.yaw) * step
        self.state.yaw = self.state.yaw + self.state.v / \
            self.wb * math.tan(steering) * step
        self.state.v = self.state.v + a * step
        self.state.w_e = self.state.w_e + self.state.w_e_dot * step
        self.state.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e

        if self.state. v > self.max_speed:
            self.state.v = self.max_speed
        elif self.state. v < self.min_speed:
            self.state.v = self.min_speed

        self.steer = steering
        self.throttle = throttle
        self.braking = braking

    def __str__(self):
        return str(self.state)
