# import numpy as np
# from tensorflow import keras
#
# model = keras.models.load_model("nn_pid.keras")
#
# W1, B1 = model.layers[0].get_weights()
# W2, B2 = model.layers[1].get_weights()
# W3, B3 = model.layers[2].get_weights()
#
# def dump(name, arr):
#     print(f"\nfloat {name}[{arr.shape[0]}][{arr.shape[1]}] = {{")
#     for r in arr:
#         print("  {", ",".join(f"{v:.6f}" for v in r), "},")
#     print("};")
#
# dump("W1", W1)
# dump("W2", W2)
# dump("W3", W3)
#
# print("\nfloat B1[] = {", ",".join(f"{v:.6f}" for v in B1), "};")
# print("float B2[] = {", ",".join(f"{v:.6f}" for v in B2), "};")
# print("float B3[] = {", ",".join(f"{v:.6f}" for v in B3), "};")

##############################################################################################
#
# import numpy as np
#
# x_scale = np.load("x_scale.npy")
# x_min   = np.load("x_min.npy")
# y_scale = np.load("y_scale.npy")
# y_min   = np.load("y_min.npy")
#
# print("float X_SCALE[] =", x_scale.tolist())
# print("float X_MIN[]   =", x_min.tolist())
# print("float Y_SCALE[] =", y_scale.tolist())
# print("float Y_MIN[]   =", y_min.tolist())
