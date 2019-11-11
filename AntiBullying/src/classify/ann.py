import numpy as np
import tensorflow as tf
import keras
from sklearn.metrics import confusion_matrix, classification_report
from . import Metrics


def ann_train_and_evaluate(training_set_samples, training_set_labels, test_set_samples, test_set_labels, xmit_conn):
  """
  Functional unit that accepts training and test sets, creates a
  fully-connected Artificial Neuron Network (ANN) classifier, trains it, and
  evaluates it. The results are communicated via the transmit-only connection
  provided as an argument. This function is meant to be used as the "target"
  of a Python :py:class:multiprocessing.Process instance. Making this
  functional unit a valid target for :py:class:multiprocessing.Process helps
  overcome the TensorFlow/Keras CUDA acceleration limitation where allocated
  GPU memory in a process is never released. Executing this logic in a
  separate process ensures that memory is released after every model run. This
  implementation also works in CPU-only runs.

  :param training_set_samples: Set of row-major samples used to train the ANN
  model.
  :type training_set_samples: numpy.ndarray
  :param training_set_labels: Set of row-major labels associated with each
  training sample.
  :type training_set_labels: numpy.ndarray
  :param test_set_samples: Set of row-major samples used to evaluate the ANN
  model.
  :type test_set_samples: numpy.ndarray
  :param test_set_labels: Set of row-major labels associated with each test
  sample.
  :type test_set_labels: numpy.ndarray
  :param xmit_conn: Transmit only connection object used to send the results
  of the model evaluation in the form of metrics.
  :type xmit_conn: multiprocessing.Connection
  """
  training_sample_count = training_set_samples.shape[0]
  n_gram_count = training_set_samples.shape[1]
  unique_class_count = np.max(np.unique(training_set_labels)) + 1
  one_hot_training_labels = np.zeros((training_sample_count, unique_class_count), np.int8)
  one_hot_training_labels[np.arange(training_sample_count), training_set_labels] = 1
  model = keras.Sequential()
  model.add(
    keras.layers.InputLayer(
      input_shape=(n_gram_count,),
      name='input_layer'
    )
  )
  model.add(
    keras.layers.Dense(
      500,
      activation=tf.nn.sigmoid,
      name='hidden_layer'
    )
  )
  model.add(
    keras.layers.Dense(
      one_hot_training_labels.shape[1],
      activation=tf.nn.sigmoid,
      name='output_layer'
    )
  )
  model.compile(
    optimizer='adam',
    loss='mse',
    metrics=['accuracy']
  )
  model.fit(training_set_samples, one_hot_training_labels, epochs=3)
  one_hot_predictions = model.predict(test_set_samples)
  predictions = np.argmax(one_hot_predictions, axis=1)
  conf_matrix = confusion_matrix(
    test_set_labels,
    predictions,
    labels=[x for x in range(unique_class_count)]
  )
  metrics_report = classification_report(
    test_set_labels,
    predictions,
    labels=[0, 1, 2, 3,],
    target_names=['Not Bullying', 'Cultural', 'Sexual', 'Personal',],
    output_dict=True
  )
  xmit_conn.send({'metrics': metrics_report, 'confusion_matrix': conf_matrix})

# vim: set ts=2 sw=2 expandtab:
