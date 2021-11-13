#!/usr/bin/env python3
import os
import sys
import time

import PIL
import PIL.Image
import cv2
import dji_osdk_ros.srv as dji_srv
import kerasncp as kncp
import numpy as np
import rospy
import tensorflow as tf
from kerasncp.tf import LTCCell
from sensor_msgs.msg import Image, Joy
from tensorflow import keras

# import logger from other ros package. Add to system path instead of including proper python lib dependency
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, "..", ".."))
from video_compression.scripts.logger_example import Logger



def dji_msg_from_velocity(vel_cmd):
    req = dji_srv.FlightTaskControlRequest()
    req.task = dji_srv.FlightTaskControlRequest.TASK_VELOCITY_AND_YAWRATE_CONTROL
    req.joystickCommand.x = vel_cmd[0][0]
    req.joystickCommand.y = vel_cmd[0][1]
    req.joystickCommand.z = vel_cmd[0][2]
    req.joystickCommand.yaw = vel_cmd[0][3]
    req.velocityControlTimeMs = 40
    return req


def load_model(model_name: str, checkpoint_name: str):
    # make sure checkpoint includes script dir so script can be run from any file
    checkpoint_path = os.path.join(script_dir, checkpoint_name)

    #last_model = tf.keras.models.load_model(checkpoint_path)
    #weights_list = last_model.get_weights()

    IMAGE_SHAPE = (144, 256, 3)
    inputs = keras.Input(shape=IMAGE_SHAPE)

    # normalization layer unssupported by version of tensorflow on drone. Data instead normalized in callback

    x = keras.layers.Conv2D(filters=16, kernel_size=(5, 5), strides=(3, 3), activation='relu')(inputs)
    x = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=8, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    # fully connected layers
    x = keras.layers.Flatten()(x)
    x = keras.layers.Dense(units=128, activation='linear')(x)
    DROPOUT = 0.0
    pre_recurrent_layer = keras.layers.Dropout(rate=DROPOUT)(x)

    if model_name == 'ncp':
        wiring = kncp.wirings.NCP(
            inter_neurons=18,  # Number of inter neurons
            command_neurons=12,  # Number of command neurons
            motor_neurons=4,  # Number of motor neurons
            sensory_fanout=6,  # How many outgoing synapses has each sensory neuron
            inter_fanout=4,  # How many outgoing synapses has each inter neuron
            recurrent_command_synapses=4,  # Now many recurrent synapses are in the
            # command neuron layer
            motor_fanin=6,  # How many incoming synapses has each motor neuron
        )
        rnn_cell = LTCCell(wiring)
        inputs_state = tf.keras.Input(shape=(rnn_cell.state_size,))

        motor_out, output_states = rnn_cell(pre_recurrent_layer, inputs_state)
        single_step_model = tf.keras.Model([inputs, inputs_state], [motor_out, output_states])

        #single_step_model.load_weights(checkpoint_name)
        single_step_model.set_weights(weights_list[3:])
        return single_step_model, rnn_cell
    else:
        raise ValueError(f"Illegal model name {model_name}")


class RNNControlNode:
    def __init__(self, path: str, log_data: bool):
        rospy.init_node("rnn_control_node")
        # base path to store all files
        self.path = path

        # state vars
        self.video_open = False
        self.close_video = True
        self.logger = Logger()
        self.path_appendix = None
        # how many seconds to wait after a change in input to start accepting more commands
        # this is here as part of the Ramin-proofing state machine
        self.delay_secs = 5
        # the last time the flight mode has changed
        self.last_change_time = rospy.get_time() - self.delay_secs
        # the most recent flight mode
        self.last_input = 8000
        # the last message printed by the state machine - used to reduce spam
        self.last_message = "starting up"

        self.log_data = log_data
        print(f"Logging for this run: {log_data}")

        # TODO: load model name and checkpoint, mean, var from rosparam
        self.mean = [0.41718618, 0.48529191, 0.38133072]
        self.variance = [.057, .05, .061]
        model_name = 'ncp'
        #checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000900_crop-0.000000_epoch-020_val_loss:0.2127_mse:0.1679_2021:09:20:02:24:31'
        checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000800_crop-0.000000_epoch-049_val_loss:0.2528_mse:0.2151_2021:11:10:19:28:04.hdf5'
        self.single_step_model, rnn_cell = load_model(model_name, checkpoint_name)
        self.hidden_state = tf.zeros((1, rnn_cell.state_size))
        print('Loaded Model')

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_client = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_client = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_client = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)



        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb)
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self.joy_cb)
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def image_cb(self, msg: Image):
        im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # resize image
        im = PIL.Image.fromarray(im_np)
        im_smaller = np.array(im.resize((256, 144), resample=PIL.Image.BILINEAR))
        im_smaller = im_smaller / 255
        im_smaller = im_smaller - self.mean
        im_smaller = im_smaller / self.variance
        im_smaller = np.expand_dims(im_smaller, 0)
        im_expanded = np.expand_dims(im_smaller, 0)

        if not self.video_open and not self.close_video:
            # start generating csv
            if self.log_data:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                self.logger.open_writer(os.path.join(self.path, "%.2f.csv" % rtime))

                # make a directory to store pngs
                self.path_appendix = '%f' % rtime
                os.mkdir(os.path.join(self.path, self.path_appendix))

            self.video_open = True
        elif self.video_open and self.close_video:
            print("ending recording")
            if self.log_data:
                # dont need to do anything in png mode since the files have already been written
                # properly close csv writer
                self.logger.close_writer()
            self.video_open = False

        if self.video_open:
            if self.log_data:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                cv2.imwrite(os.path.join(self.path, self.path_appendix, ('%.3f' % rtime) + ".png"), im_smaller)
                # add newest state for this frame to the csv
                self.logger.write_state(rostime)

            # run inference on im_expanded
            vel_cmd, self.hidden_state = self.single_step_model([im_expanded, self.hidden_state])
            print(vel_cmd)

            ca_req = dji_srv.ObtainControlAuthorityRequest()
            ca_req.enable_obtain = True
            ca_res = self.ca_client.call(ca_req)
            print('\n\nca_res: ', ca_res, '\n\n')


            joymode_req = dji_srv.SetJoystickModeRequest()
            joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
            joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
            joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
            joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
            joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
            res1 = self.joystick_mode_client.call(joymode_req) 
            print('joymode response: ', res1)


            # construct dji velocity command
            #req = dji_msg_from_velocity(vel_cmd)
            #res = self.velocity_service.call(req)
            #print(res)




            joyact_req = dji_srv.JoystickActionRequest()
            joyact_req.joystickCommand.x = vel_cmd[0][0]
            joyact_req.joystickCommand.y = vel_cmd[0][1]
            joyact_req.joystickCommand.z = vel_cmd[0][2]
            joyact_req.joystickCommand.yaw = vel_cmd[0][3]
            res2 = self.joystick_action_client.call(joyact_req)
            print('Joyact response: ', res2)

            #t0 = time.time()
            #while (time.time() - t0 < 1000):
            #    res2 = self.joystick_action_client.call(joyact_req)
            #    print('Joyact response: ', res2)



    # T -8000 (left)
    # P 8000  (center)
    # S 0     (right)
    def joy_cb(self, msg):
        """
        Processes the joystick message to detect mode changes to decide what to do with recording

        Moving the mode switch right turns on recording, and moving it to the left turns off recording
        There is also a delay that starts counting any time the input changes
        This way, after a command is given the mode switch has to be in a steady state for some time before
        new commands can be given, making it ok to accidentally switch it around a bunch while trying to get the switch to the center

        """
        current_input = msg.axes[4]

        # whether or not we're waiting for a delay
        waiting = abs(rospy.get_time() - self.last_change_time) < self.delay_secs

        message = ""

        if current_input == -8000 and not self.close_video and not waiting:
            self.close_video = True
            message = "close video command sent"
        elif current_input == 0 and self.close_video and not waiting:
            self.close_video = False
            message = "start video command sent"
        elif not waiting:
            message = "accepting commands"
        else:
            message = "waiting"

        if current_input != self.last_input:
            self.last_change_time = rospy.get_time()
            message += "\nwaiting %.2f seconds before allowing next command" % self.delay_secs

        # only send changed messages to avoid spam
        if self.last_message != message:
            print(message)
            self.last_message = message

        self.last_input = current_input


if __name__ == "__main__":
    path_param = rospy.get_param("~path", default="/home/dji/flash/")
    log_data = rospy.get_param("~log_data", default=False)
    node = RNNControlNode(path_param, log_data)
