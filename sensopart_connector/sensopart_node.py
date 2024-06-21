from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensopart_connector.sensopart import SensoPart
from std_srvs.srv import Trigger
from sensopart_interfaces.msg import Job
from sensopart_interfaces.srv import ExtendedTrigger, GetImage, GetJobs, SetJob, Calibration, TriggerRobotics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class SensopartNode(Node):

    PARAM_IP = 'ip'
    PARAM_AUTO_CONNECT = 'auto_connect'

    PARAM_TYPES = {
        PARAM_IP: Parameter.Type.STRING,
        PARAM_AUTO_CONNECT: Parameter.Type.BOOL
    }

    def __init__(self):
        super().__init__("sensopart_connector")
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.ip = self.declare_parameter(self.PARAM_IP, '172.31.1.198')
        self.auto_connect = self.declare_parameter(self.PARAM_AUTO_CONNECT, True)
        self.publish_image_on_trigger = self.declare_parameter('publish_image_on_trigger', True)
        self.add_on_set_parameters_callback(self.parameter_changed)

        self.get_logger().info(
            f"Current parameters:\n"
            f"  address: {self.ip.value}:{SensoPart.REQUEST_PORT}\n"
            f"  auto-connect: {self.auto_connect.value}")

        self.cv_bridge = CvBridge()

        log = rclpy.logging.get_logger("sensorpart")
        self.sensopart = SensoPart(self.ip.value, log)

        self.image_pub = self.create_publisher(Image, f'{self.get_name()}/image', 10)

        # Connect/disconnect services
        self.connect_service = self.create_service(
            Trigger, f'{self.get_name()}/connect', self.connect_callback)
        self.disconnect_service = self.create_service(
            Trigger, f'{self.get_name()}/disconnect', self.disconnect_callback)

        # Trigger services
        self.trigger_service = self.create_service(
            Trigger, f'{self.get_name()}/trigger', self.trigger_callback)
        self.extended_trigger_service = self.create_service(
            ExtendedTrigger, f'{self.get_name()}/extended_trigger', self.extended_trigger_callback)

        # Trigger with position service
        self.trigger_with_position_service = self.create_service(
            TriggerRobotics, f'{self.get_name()}/trigger_with_position', self.trigger_with_position_callback)
        
        # Image service
        self.get_image_service = self.create_service(
            GetImage, f'{self.get_name()}/get_image', self.get_image_callback)

        # Job services
        self.get_job_list_service = self.create_service(
            GetJobs, f'{self.get_name()}/get_job_list', self.get_job_list_callback)
        self.change_job_service = self.create_service(
            SetJob, f'{self.get_name()}/change_job', self.change_job_callback)
        self.change_job_permanent_service = self.create_service(
            SetJob, f'{self.get_name()}/change_job_permanent', self.change_job_permanent_callback)
        
        # Calibration services
        self.calibrate_service = self.create_service(
            Calibration, f'{self.get_name()}/calibrate', self.calibrate_callback)
        

        # Auto-connect
        if self.auto_connect.value:
            self.get_logger().info("Auto connect is on, attempting to connect to camera...")
            success, msg = self.sensopart.connect()
            self.get_logger().info(msg)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def parameter_changed(self, parameters: List[Parameter]) -> SetParametersResult:
        ip_changed = False
        for parameter in parameters:
            if parameter.name in self.PARAM_TYPES:
                actual = parameter.type_
                expected = self.PARAM_TYPES[parameter.name]
                if actual != expected:
                    msg = f"Invalid type: {actual}. Expected: {expected}."
                    self.get_logger().error(msg)
                    return SetParametersResult(successful=False, reason=msg)

            if parameter.name == self.PARAM_IP:
                self.get_logger().info(f"IP updated from: {self.ip.value} to: {parameter.value}")
                self.ip = parameter
                ip_changed = True

            if parameter.name == self.PARAM_AUTO_CONNECT:
                self.get_logger().info(
                    f"Auto-connect changed from: {self.auto_connect.value} to: {parameter.value}")
                self.auto_connect = parameter

        if ip_changed:
            self.sensopart.disconnect()
            self.sensopart = SensoPart(self.ip.value, self.sensopart.log)
            if self.auto_connect.value:
                success, msg = self.sensopart.connect()
                return SetParametersResult(successful=success, reason=msg)

        return SetParametersResult(successful=True)
    
    def calibrate_callback(self, request: Calibration.Request, response: Calibration.Response):
        # Init calibration
        if request.init:
            success = self.sensopart.calibration_init()
            self.get_logger().info(f"Calibration init: {success}")
            response.success = success
            return response
        # Compute calibration
        elif request.compute:
            success = self.sensopart.run_calibration()
            response.success = success

            return response
        # Add the Pose to the calibration
        else:            
            success = self.sensopart.add_calibration_image(request.pose,request.first)
            if success > 0:
                response.success = True
                response.image_number = success
            else:
                response.success = False
            return response

    def connect_callback(self, _: Trigger.Request, response: Trigger.Response):
        return self.trigger_helper(self.sensopart.connect, response)

    def disconnect_callback(self, _: Trigger.Request, response: Trigger.Response):
        return self.trigger_helper(self.sensopart.disconnect, response)

    def trigger_callback(self, _: Trigger.Request, response: Trigger.Response):
        response = self.trigger_helper(self.sensopart.trigger, response)
        self.publish_image()
        return response

    def trigger_helper(self, method, response: Trigger.Response) -> Trigger.Response:
        success, message = method()
        response.success = success
        response.message = message
        return response

    def extended_trigger_callback(self,
                                  request: ExtendedTrigger.Request,
                                  response: ExtendedTrigger.Response):
        success, message, passed, mode = self.sensopart.extended_trigger(request.data)
        response.success = success
        response.message = message
        response.passed = passed
        response.mode = mode
        self.publish_image()
        return response
    
    # Trigger for getting an image from the camera with the position of the robot
    def trigger_with_position_callback(self, request: TriggerRobotics.Request, response: TriggerRobotics.Response):
        response.success, response.message = self.sensopart.trigger_with_position(request.pose, request.trigerid)
        return response
    

    def get_image_callback(self, request: GetImage.Request, response: GetImage.Response):
        if request.trigger:
            success, message = self.sensopart.trigger()
            if not success:
                response.success = success
                response.message = message
                return response

        success, message, image = self.get_image()
        response.success = success
        response.message = message
        response.image = image
        return response

    def publish_image(self):
        if self.publish_image_on_trigger.value:
            success, _, image = self.get_image()
            if success:
                self.image_pub.publish(image)

    def get_image(self) -> (bool, str, Image):
        try:
            success, message, cv2_image = self.sensopart.get_image()
            image = self.cv_bridge.cv2_to_imgmsg(cv2_image)
            self.get_logger().info(f"Image ({image.width}:{image.height}, {image.encoding})")
            return True, message, image
        except CvBridgeError as e:
            self.get_logger().warning(f"Error converting image: {e}")
            return False, f"Error converting image: {e}", Image()

    def get_job_list_callback(self, _: GetJobs.Request, response: GetJobs.Response):
        success, msg, jobs = self.sensopart.get_job_list()
        response.success = success
        response.message = msg
        response.jobs = [Job(**job.__dict__) for job in jobs]
        return response

    def change_job_callback(self, request: SetJob.Request, response: SetJob.Response):
        success, msg = self.sensopart.change_job(request.job)
        response.success = success
        response.message = msg
        return response

    def change_job_permanent_callback(self, request: SetJob.Request, response: SetJob.Response):
        success, msg = self.sensopart.change_job_permanent(request.job)
        response.success = success
        response.message = msg
        return response

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down...")
        del self.sensopart
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensopartNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
