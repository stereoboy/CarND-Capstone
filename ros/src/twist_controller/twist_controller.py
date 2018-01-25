from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704 # 1 MPH = 0.44704 m/s

K_P = 0.3
K_I = 0.005 # K_I is important because this throttle control is biased!
K_D = 0.1

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                    wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        self.lowpass_filter = LowPassFilter(1.0, 3.0)
        self.reset_throttle_ctrl()
        self.yaw_ctrl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.last_stamp = None

    def reset_throttle_ctrl(self):
        self.throttle_ctrl = PID(kp=K_P, ki=K_I, kd=K_D, mn=self.decel_limit, mx=self.accel_limit)

    def control(self, linear_velocity, angular_velocity, current_velocity, current_stamp, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if self.last_stamp is None:
            sample_time = 1.0/50.0 # 50Hz
        else:
            sample_time = (current_stamp - self.last_stamp)
        self.last_stamp = current_stamp
        rospy.loginfo("error: {} - {} = {}".format(linear_velocity, current_velocity, linear_velocity - current_velocity))
        rospy.loginfo("sample_time: {}".format(sample_time))

        error = linear_velocity - current_velocity
        accel = self.throttle_ctrl.step(error, sample_time)
        rospy.loginfo("before lowpassfilter: {}".format(accel))
        accel = self.lowpass_filter.filt(accel)
        rospy.loginfo("after lowpassfilter: {}".format(accel))

        if accel > 0.0:
            throttle = accel
            if linear_velocity == 0.0:
                throttle = 0.0
            brake = 0.0
            rospy.loginfo("throttle: {}".format(throttle))
        else:
            if accel < -self.brake_deadband:
                brake = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*abs(accel)*self.wheel_radius
            else:
                brake = 0.0
            throttle = 0.0
            rospy.loginfo("brake: {}".format(brake))

            # Heuristic. if we need decel curve, we need to igore integral value of PID.
            self.reset_throttle_ctrl()

        rospy.loginfo("\t\t\t\t\t\t\t\t\t(throttle, brake): {}\t\t\t{}".format(throttle, brake))
        steer = self.yaw_ctrl.get_steering(linear_velocity, angular_velocity, current_velocity)
        return throttle, brake, steer
