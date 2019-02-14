import rospy
import dji_sdk
from sensor_msgs.msg import Joy
from enum import Enum

class dji_m100_commands():
    
    def __init__(self, nh):

        # PUB
        self.generic_pub = rospy.Publisher('/dji_sdk/flight_control_setpoint_generic', Joy)
        self.enu_pos_yaw_pub = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUposition_yaw', Joy)
        self.enu_vel_yaw_rate_pub = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy)
        self.roll_pitch_yaw_rate_height_pub = rospy.Publisher('/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition', Joy)

        # SERVICES
        self.set_local_pos_ref = rospy.ServiceProxy('/dji_sdk/set_local_pos_ref', dji_sdk.srv.SetLocalPosRef)
        self.waypoint_task_upload = rospy.ServiceProxy('/dji_sdk/mission_waypoint_upload', dji_sdk.srv.MissionWaypointTask)
        self.toggle_control_authority = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', dji_sdk.srv.SDKControlAuthority)
        

    def setpoint_generic(self, a0, a1, a2, a3, flag):
        """ 
            General setpoint where axes[0] to axes[3] stores set-point data for the 2
            horizontal channels, the vertical channel, and the yaw channel, respectively.
            The meaning of the set-point data will be interpreted based on the control 
            flag which is stored in axes[4].

            The control flag is an UInt8 variable that dictates how the inputs are 
            interpreted by the flight controller. It is the bitwise OR of 5 separate 
            flags defined as enums in dji_sdk.h, including Horizontal, Vertical, Yaw,
            Coordinate Frame, and the Breaking Mode.
            
            The final flag is (VERTICAL       |
                               HORIZONTAL     |
                               YAW_RATE       |
                               HORIZONTAL_REF |
                               STABLE_ENABLED)

        """
        self.generic_pub.publish(Joy([a1, a2, a3, flag]))

    def setpoint_FRUposition_yaw(self, x, y, z, yaw):
        """ Command the X, Y position offset, Z position (height) and yaw angle in FRU ground frame """
        flag = (Flags.HORIZONTAL_POSITION |
                Flags.VERTICAL_POSITION   |
                Flags.YAW_ANGLE           |
                Flags.HORIZONTAL_BODY     |
                Flags.STABLE_ENABLE) 
        self.setpoint_generic(x, y, z, yaw, flag)
    
    def setpoint_ENUposition_yaw(self, x, y, z, yaw):
        """ Command the X, Y position offset, Z position (height) and yaw angle in ENU ground frame """
        self.enu_pos_yaw_pub.publish(Joy([x,y,z,yaw]))

    def setpoint_ENUvelocity_yawrate(self, dx, dy, dz, d_yaw):
        """ Command the X, Y, Z velocity in ENU ground frame and yaw rate. """
        self.enu_vel_yaw_rate_pub.publish(Joy([dx, dy, dz, d_yaw]))

    def setpoint_roll_pitch_yaw_rate_height(self, roll, pitch, height, d_yaw):
        """ Command the roll pitch angle, height, and yaw rate. """
        self.roll_pitch_yaw_rate_height_pub.publish(Joy([roll, pitch, height, d_yaw]))

    def set_local_position(self):
        local_pos_ref_req = dji_sdk.srv.SetLocalPosRef()
        response = self.set_local_pos_ref.call(local_pos_ref_req)
        return response.result

    def upload_waypoints(self, waypoints, **kwargs):
        request = dji_sdk.msg.MissionWaypointTask(**kwargs)
        waypoint = dji_sdk.msg.MissionWaypoint()
        request.mission_waypoint = [waypoint]
        response = self.waypoint_task_upload(request)
        return response.result

    def toggle_control_authority(self, control_enable):
        # RELEASE_CONTROL = 0
        # REQUEST_CONTROL = 1
        response = self.toggle_control_authority(control_enable)
        return response.result

class Flags(Enum):
    """ Flags options to control setpoint options """
    HORIZONTAL_ANGLE        = 0x00
        # - Set the control-mode to control pitch & roll
        #   angle of the vehicle.
        # - Need to be referenced to either the ground or
        #   body frame by HorizontalCoordinate setting.
        # - Limit: 35 degree
    HORIZONTAL_VELOCITY     = 0x40
        # - Set the control-mode to control horizontal
        #   vehicle velocities.
        # - Need to be referenced to either the ground
        #   or body frame by HorizontalCoordinate setting.
        # - Limit: 30 m/s
    HORIZONTAL_POSITION     = 0x80
        # - Set the control-mode to control position
        #   offsets of pitch & roll directions
        # - Need to be referenced to either the ground
        #   or body frame by HorizontalCoordinate setting.
        # - Limit: N/A
    HORIZONTAL_ANGULAR_RATE = 0xC0
        # - Set the control-mode to control rate of
        # change of the vehicle's attitude
        # - Need to be referenced to either the ground
        # or body frame by HorizontalCoordinate setting.
        # - Limit: 150.0 deg/s
    VERTICAL_VELOCITY       = 0x00
        # - Set the control-mode to control the vertical
        # speed of UAV, upward is positive
        # - Limit: -5 to 5 m/s
    VERTICAL_POSITION       = 0x10
        # - Set the control-mode to control the height of UAV
        # - Limit: 0 to 120 m
    VERTICAL_THRUST         = 0x20
        # - Set the control-mode to directly control the thrust
        # - Range: 0% to 100%
    YAW_ANGLE               = 0x00
        # - Set the control-mode to control yaw angle.
        # - Yaw angle is referenced to the ground frame.
        # - In this control mode, Ground frame is enforeced in Autopilot.
    YAW_RATE                = 0x08
        # - Set the control-mode to control yaw angular velocity.
        # - Same reference frame as YAW_ANGLE.
        # - Limite: 150 deg/s
    HORIZONTAL_GROUND       = 0x00
        # - Set the x-y of ground frame as the horizontal frame (NEU)
        #   North-East for horizontal
        #   CW = + for yaw
        #   U  = + for z
    HORIZONTAL_BODY         = 0x02
        # - Set the x-y of body frame as the horizontal frame (FRU)
        #   Forward-Right for horizontal
        #   CW = + for yaw
        #   U  = + for z
    STABLE_DISABLE          = 0x00
    STABLE_ENABLE           = 0x01


"""
enum M100FlightStatus
{
  M100_STATUS_ON_GROUND        = DJI::OSDK::VehicleStatus::M100FlightStatus::ON_GROUND_STANDBY,
  M100_STATUS_TAKINGOFF        = DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF,
  M100_STATUS_IN_AIR           = DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY,
  M100_STATUS_LANDING          = DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING,
  M100_STATUS_FINISHED_LANDING = DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING
};

"""

class M100FlightStatus(Enum):
    M100_STATUS_ON_GROUND        = 1
    M100_STATUS_TAKINGOFF        = 2
    M100_STATUS_IN_AIR           = 3
    M100_STATUS_LANDING          = 4
    M100_STATUS_FINISHED_LANDING = 5

if __name__ == "__main__":
    pass
