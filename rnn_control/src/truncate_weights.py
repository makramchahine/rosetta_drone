import h5py
import kerasncp as kncp
import tensorflow as tf
from keras.saving.hdf5_format import load_subset_weights_from_hdf5_group
from kerasncp.tf import LTCCell
from tensorflow import keras

def truncate_weights(filepath, discard_up_to):
    DROPOUT = 0.1
    IMAGE_SHAPE = (144, 256, 3)
    inputs = keras.Input(shape=IMAGE_SHAPE)

    rescaling_layer = keras.layers.experimental.preprocessing.Rescaling(1. / 255)
    normalization_layer = keras.layers.experimental.preprocessing.Normalization(
        mean=[0.41718618, 0.48529191, 0.38133072],
        variance=[.057, .05, .061])
    x = rescaling_layer(inputs)
    x = normalization_layer(x)
    my_input_model = keras.Model(inputs=inputs, outputs=x)

    model = keras.models.Sequential()
    model.add(my_input_model)

    # # Conv Layers for 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000900_crop-0.000000_epoch-047_val_loss:0.2574_mse:0.0683_2021:09:24:09:22:31.hdf5'
    # x = keras.layers.Conv2D(filters=16, kernel_size=(5, 5), strides=(3, 3), activation='relu')(x)
    # x = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    # x = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    # x = keras.layers.Conv2D(filters=8, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)

    x = keras.layers.Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), activation='relu')(x)
    x = keras.layers.Conv2D(filters=16, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)

    # fully connected layers
    x = keras.layers.Flatten()(x)
    x = keras.layers.Dense(units=128, activation='linear')(x)
    pre_recurrent_layer = keras.layers.Dropout(rate=DROPOUT)(x)

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
    rnn_cell = LTCCell(wiring, ode_unfolds=6)
    inputs_state = tf.keras.Input(shape=(rnn_cell.state_size,))

    motor_out, output_states = rnn_cell(pre_recurrent_layer, inputs_state)
    single_step_model = tf.keras.Model([inputs, inputs_state], [motor_out, output_states])

    single_step_model.load_weights(filepath)

    # build model no norm
    inputs = keras.Input(shape=IMAGE_SHAPE)
    x_no_norm = keras.layers.Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation='relu')(inputs)
    x_no_norm = keras.layers.Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x_no_norm)
    x_no_norm = keras.layers.Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), activation='relu')(x_no_norm)
    x_no_norm = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), activation='relu')(x_no_norm)
    x_no_norm = keras.layers.Conv2D(filters=16, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x_no_norm)
    x_no_norm = keras.layers.Flatten()(x_no_norm)
    x_no_norm = keras.layers.Dense(units=128, activation='linear')(x_no_norm)
    pre_recurrent_layer_no_norm = keras.layers.Dropout(rate=DROPOUT)(x_no_norm)
    rnn_cell = LTCCell(wiring, ode_unfolds=6)
    inputs_state = tf.keras.Input(shape=(rnn_cell.state_size,))

    motor_out, output_states = rnn_cell(pre_recurrent_layer_no_norm, inputs_state)
    single_step_model_no_norm = tf.keras.Model([inputs, inputs_state], [motor_out, output_states])

    # load weights into ncp model
    single_step_model.load_weights(filepath)
    weights_list = single_step_model.get_weights()

    # truncate and save
    weights_truncated = weights_list[discard_up_to:]
    single_step_model_no_norm.set_weights(weights_truncated)
    single_step_model_no_norm.save_weights("test.hdf5")

truncate_weights("models/ncp1.hdf5", 3)
# truncate_weights("models/rev-0_model-ncp_seq-64_opt-adam_lr-0.000800_crop-0.000000_epoch-049_val_loss:0.2528_mse:0.2151_2021:11:10:19:28:04.hdf5", 3)