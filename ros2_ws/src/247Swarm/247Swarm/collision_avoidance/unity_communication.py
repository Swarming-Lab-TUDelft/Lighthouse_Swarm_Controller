import zmq
import json
import time

from typing import List

unity_connection_time_out_ = 5

class Serializable:
    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

    @classmethod
    def from_json(cls, json_string):
        raise NotImplementedError("from_json() method should be implemented in each subclass")

class Vehicle_t(Serializable):
    def __init__(self):
        self.ID = ""
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.desiredVelocity = [0.0, 0.0, 0.0]
        self.rotation = [0.0, 0.0, 0.0, 1.0]
        self.size = [1.0, 1.0, 1.0]
        self.cameras = []
        self.lidars = []
        self.has_collision_check = True

class SettingsMessage_t(Serializable):
    def __init__(self):
        #self.scene_id = None  # Replace with the appropriate scene ID, e.g., UnityScene.WAREHOUSE
        self.vehicles = []
        self.objects = []

class PubMessage_t(Serializable):
    def __init__(self):
        self.frame_id = 0
        self.vehicles = []
        self.objects = []

class Sub_Vehicle_t(Serializable):
    def __init__(self, ID=None, velocityCommand=None, collision=False, lidar_ranges=None):
        self.ID = ID if ID is not None else ""
        self.velocityCommand = velocityCommand if velocityCommand is not None else [0,0,0]
        self.collision = collision
        self.lidar_ranges = lidar_ranges if lidar_ranges is not None else []

    @classmethod
    def from_json(cls, json_string):
        return cls(**json.loads(json_string))

class SubMessage_t(Serializable):
    def __init__(self, frame_id=0, sub_vehicles=None):
        self.frame_id = frame_id
        self.sub_vehicles = sub_vehicles if sub_vehicles is not None else []

    @classmethod
    def from_json(cls, json_string):
        data = json.loads(json_string)
        #print(f"loads json: {data}")
        frame_id = data['frame_id']
        sub_vehicles = [Sub_Vehicle_t.from_json(json.dumps(sv)) for sv in data['pub_vehicles']]
        return cls(frame_id=frame_id, sub_vehicles=sub_vehicles)

def vectorRos2Unity(ros_pos_vec) -> List[float]:
    # Position Vector from ROS coordinate system (right hand) to Unity coordinate system (left hand)
    unity_position = [-ros_pos_vec[1], ros_pos_vec[2], ros_pos_vec[0]] #Flightmare (old): [ros_pos_vec[0], ros_pos_vec[2], ros_pos_vec[1]] 

    return unity_position

def vectorUnity2Ros(unity_pos_vec) -> List[float]:
    # Position Vector from Unity coordinate system (left hand) to ROS coordinate system (right hand)
    ros_position = [unity_pos_vec[2], -unity_pos_vec[0], unity_pos_vec[1]]

    return ros_position

def quaternionRos2Unity(ros_quat):
    # Convert ROS quaternion to left hand coordinate system
    unity_quat = [ros_quat[1], -ros_quat[2], -ros_quat[0], ros_quat[3]]

    return unity_quat

def scaleRos2Unity(ros_scale_vec) -> List[float]:
    # Scale Vector from ROS coordinate system (right hand) to Unity coordinate system (left hand)
    unity_scale = [ros_scale_vec[1], ros_scale_vec[2], ros_scale_vec[0]]

    return unity_scale

class UnityBridge:
    def __init__(self):
        self.client_address_ = "tcp://*"
        self.pub_port_ = "10253"
        self.sub_port_ = "10254"
        self.num_frames_ = 0
        self.last_downloaded_utime_ = 0
        self.last_download_debug_utime_ = 0
        self.u_packet_latency_ = 0
        self.unity_ready_ = False

        self.settings = SettingsMessage_t()
        self.pub_msg = PubMessage_t()

        # initialize connections upon creating unity bridge
        self.initializeConnections()

    def initializeConnections(self):
        context = zmq.Context()
        print("Initializing ZMQ connection!")

        # create and bind an upload socket
        self.pub_ = context.socket(zmq.PUB)
        self.pub_.setsockopt(zmq.SNDHWM, 0)
        self.pub_.bind(self.client_address_ + ":" + self.pub_port_)

        # create and bind a download_socket
        self.sub_ = context.socket(zmq.SUB)
        self.sub_.setsockopt(zmq.RCVHWM, 0) #6 is default
        self.sub_.bind(self.client_address_ + ":" + self.sub_port_)

        # subscribe all messages from ZMQ
        self.sub_.subscribe("")

        print("Initializing ZMQ connections done!")
        return True
    
    def connectUnity(self):
        time_out_count = 0
        sleeptime = 0.2
        # try to connect unity
        print("Trying to Connect Unity.")
        print("[", end="")
        while not self.unity_ready_:
            # if time out
            if time_out_count / 1e6 > unity_connection_time_out_:
                print("]")
                print(
                    "Unity Connection time out! Make sure that Unity Standalone "
                    "or Unity Editor is running the Flightmare.")
                return False
            # initialize Scene settings
            self.sendInitialSettings()
            # check if setting is done
            self.unity_ready_ = self.handleSettings()
            # sleep
            time.sleep(sleeptime)
            # increase time out counter
            time_out_count += sleeptime
            # print something
            print(".", end="")
            #sys.stdout.flush()
        print("Flightmare Unity is connected.")
        return self.unity_ready_

    def disconnectUnity(self):
        self.unity_ready_ = False
        # create new message object
        self.pub_.close()
        self.sub_.close()
        return True
    
    def sendInitialSettings(self):
        # add topic header
        topic_header = b"Pose"
        # create JSON object for initial settings
        json_mesg = self.settings.to_json().encode()
        # send message without blocking
        self.pub_.send_multipart([topic_header, json_mesg], flags=zmq.NOBLOCK)
        return True

    def handleSettings(self):
        done = False
        # create new message object
        try:
            msg = self.sub_.recv_multipart(zmq.NOBLOCK)
            # Unpack message metadata
            if msg:
                metadata_string = msg[0].decode()
                # Parse metadata
                if len(json.loads(metadata_string)) > 1:
                    return False  # hack
                done = json.loads(metadata_string)["ready"]
            
        except zmq.Again:
            # No message received, handle the situation accordingly
            #print("No message received")
            pass
        
        return done
    
    def updateQuadrotor(self, quad):
        #search by id
        for vehicle in self.pub_msg.vehicles:
            if vehicle.ID == quad.ID:
                vehicle.position = vectorRos2Unity(quad.position)
                vehicle.velocity = vectorRos2Unity(quad.velocity)
                vehicle.desiredVelocity = vectorRos2Unity(quad.desiredVelocity)
                vehicle.rotation = quaternionRos2Unity(quad.rotation)
                vehicle.size = scaleRos2Unity(quad.size)
                return True

    def toUnity(self, frame_id, logger) -> bool:
        # add topic header
        self.pub_msg.frame_id = frame_id
        topic_header = b"Pose"
        # create JSON object for initial settings
        json_mesg = self.pub_msg.to_json().encode()
        #logger.info(f"Sending to Unity frame_id: {frame_id}")
        # send message without blocking
        self.pub_.send_multipart([topic_header, json_mesg], flags=zmq.NOBLOCK)
        return True
    

    def fromUnity(self, frameID, logger):

        commandList = {}
            
        recieved = False
        while not recieved:
            # Unpack message metadata
            try:
                msg = self.sub_.recv_multipart(zmq.NOBLOCK)

                # Unpack message metadata
                if msg:
                    self.subMessage = SubMessage_t.from_json(msg[0].decode())
                    #logger.info(f"Message: {msg}, frameID: {frameID}")
                    if self.subMessage.frame_id == frameID:
                        #logger.info("Recieved correct message from Unity")
                        recieved = True
                        for vehicle in self.subMessage.sub_vehicles:
                            commandList[vehicle.ID] = vectorUnity2Ros(vehicle.velocityCommand)
                    else:
                        #logger.info(f"Recieved message from Unity with wrong frameID: {self.subMessage.frame_id}, Gazebo frameID: {frameID}")
                        self.toUnity(frameID, logger)
                        
            except zmq.Again:
                # No message received, handle the situation accordingly
                self.toUnity(frameID, logger)
        
        return commandList

    def add_quadrotor(self, quad) -> bool:
        vehicle = Vehicle_t()

        vehicle.ID = quad.ID
        vehicle.position = quad.position
        vehicle.rotation = quad.rotation
        vehicle.size = quad.size

        self.settings.vehicles.append(vehicle)
        self.pub_msg.vehicles.append(vehicle)
        return True

