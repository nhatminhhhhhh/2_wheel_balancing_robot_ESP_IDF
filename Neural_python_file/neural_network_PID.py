import numpy as np
import pandas as pd
from tensorflow import keras
from sklearn.preprocessing import MinMaxScaler

# Load data
#data = pd.read_csv("fuzzy_log.csv")
data = pd.read_csv(r"C:\Users\ROG STRIX\PycharmProjects\Jetson_Project\fuzzy_log.csv")


X = data[['e', 'de']].values
Y = data[['Kp', 'Kd']].values

# Normalize
x_scaler = MinMaxScaler()
y_scaler = MinMaxScaler()
Xn = x_scaler.fit_transform(X)
Yn = y_scaler.fit_transform(Y)

# Neural Network
model = keras.Sequential([
    keras.layers.Dense(8, activation='relu', input_shape=(2,)),
    keras.layers.Dense(8, activation='relu'),
    keras.layers.Dense(2, activation='linear')
])

model.compile(optimizer='adam', loss='mse')
model.fit(Xn, Yn, epochs=200, batch_size=32, verbose=1)

# Save model + scaler
model.save("nn_pid.keras")
np.save("x_scale.npy", x_scaler.scale_)
np.save("x_min.npy", x_scaler.min_)
np.save("y_scale.npy", y_scaler.scale_)
np.save("y_min.npy", y_scaler.min_)

########################################################################################################################
