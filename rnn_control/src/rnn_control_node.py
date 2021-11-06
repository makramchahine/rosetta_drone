#!/usr/bin/python
import numpy as np
import rospy
import dji_osdk_ros.srv as dji_srv
from sensor_msgs.msg import Image


def image_cb(msg):
    global hidden_state, single_step_model

    im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    im = Image.fromarray(im_np)
    im_smaller = im.resize((256, 144), resample=PIL.Image.BILINEAR)

    # run inference on im_smaller
    vel_cmd, hidden_state = single_step_model([im_smaller, hidden])

    req = dji_srv.FlightTaskControlReq()
    req.task = dji_srv.FlightTaskControl.TASK_VELOCITY_AND_YAWRATE_CONTROL
    req.joystickCommand.x = vel_cmd[0]
    req.joystickCommand.y = vel_cmd[1]
    req.joystickCommand.z = vel_cmd[2]
    req.joystickCommand.yaw = vel_cmd[3]
    req.velocityControlTimeMs = 40

    velocity_service.call(req)


    last_model = tf.keras.models.load_model(checkpoint)
    weights_list = last_model.get_weights()

model_name = 'ncp'
checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000900_crop-0.000000_epoch-020_val_loss:0.2127_mse:0.1679_2021:09:20:02:24:31'

inputs = keras.Input(shape=IMAGE_SHAPE)

rescaling_layer = keras.layers.experimental.preprocessing.Rescaling(1. / 255)
normalization_layer = keras.layers.experimental.preprocessing.Normalization(mean=[0.41718618, 0.48529191, 0.38133072],
                                                                            variance=[.057, .05, .061])
x = rescaling_layer(inputs)
x = normalization_layer(x)

x = keras.layers.Conv2D(filters=16, kernel_size=(5, 5), strides=(3, 3), activation='relu')(x)
x = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
x = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
x = keras.layers.Conv2D(filters=8, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
# fully connected layers
x = keras.layers.Flatten()(x)
x = keras.layers.Dense(units=128, activation='linear')(x)
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

    # single_step_model.load_weights(checkpoint)
    single_step_model.set_weights(weights_list)


hidden_state = tf.zeros((1, rnn_cell.state_size))


rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, image_cb)
velocity_service = rospy.ServiceProxy('/flight_task_control')



rospy.spin()
