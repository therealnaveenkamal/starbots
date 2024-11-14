from tensorflow.keras.models import load_model
import cv2
import numpy as np


path = "/home/user/ros2_ws/src/cone_detection/src/cone_detection/"

model = load_model(path+'starbots_cup_detection.h5')

# Load and preprocess the new image
new_image_path = path+'cup3.jpg'
new_image = cv2.imread(new_image_path)
new_image = new_image / 255.0  # Normalize the image

# Reshape the image to match the input shape expected by the model
new_image = np.expand_dims(new_image, axis=0)

# Use the trained model to predict the bounding box coordinates
predicted_bbox = model.predict(new_image)

# Unpack the predicted bounding box coordinates
predicted_w, predicted_h, predicted_x, predicted_y = predicted_bbox[0]

pred_image = cv2.imread(new_image_path)
w = int(predicted_w)
h = int(predicted_h)
x = int(predicted_x)
y = int(predicted_y)

centroid_x = (x + x + w) // 2
centroid_y = (y + y + h) // 2

cv2.rectangle(pred_image, (centroid_x, centroid_y), (centroid_x+1, centroid_y+1), (0, 0, 255), 2)
cv2.rectangle(pred_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

cv2.imshow("Image with Bounding Box", pred_image)
cv2.waitKey(0)
cv2.destroyAllWindows()