import os
import cv2
import csv
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
from datetime import datetime as dt

# Start Time
t_start = dt.now()
print("Start Time:", str(t_start))

# Define a function to load and preprocess images
def load_images(image_folder):
    images = []
    annotations = []

    # Loop through each image file in the folder
    for filename in os.listdir(image_folder):
        if filename.endswith(".jpg") and filename.startswith("color_image"): # Adjust the file extensions as needed
            # Parse information from the filename
            parts = filename.split("_")
            y = parts[3]
            x = parts[4]
            w = parts[5]
            h = parts[6].split(".")[0]
            image_path = os.path.join(image_folder, filename)

            # Load image
            image = cv2.imread(image_path)
            if image is None:
                print(f"Could not read image: {image_path}")
                continue

            # Append image and annotations
            images.append(image)
            annotations.append([int(y), int(x), int(w), int(h)])

    return np.array(images), np.array(annotations)

# Load images from a folder
image_folder = '/home/gkk-ros/cup_samples_generator/generated_images'
images, annotations = load_images(image_folder)

images = images / 255.0

# Split dataset into training and testing sets
# (you can add validation set too if needed)
train_images = images[:2000]
train_annotations = annotations[:2000]
test_images = images[500:]
test_annotations = annotations[500:]

# Define your neural network architecture
def create_model():
    model = models.Sequential([
        layers.Conv2D(32, (3, 3), activation='relu',
                      input_shape=(152, 270, 3)),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dense(4)  # 4 output values: y, x, w, h
    ])
    return model

# Compile the model
model = create_model()
model.compile(optimizer='adam',
              loss='mean_squared_error',  # Use appropriate loss function for regression
              metrics=['accuracy'])

# Train the model
model.fit(train_images, train_annotations, epochs=50, batch_size=32)

# Evaluate the model
test_loss, test_acc = model.evaluate(test_images, test_annotations)
print('Test accuracy:', test_acc)

# Make predictions
predictions = model.predict(test_images)

# Save Trained Model
model.save('starbots_cup_detection.h5')

# End Time
t_end = dt.now()
print("End Time:", str(t_end))

# Process Time
t_process = (t_end - t_start)
print("Process Time:", str(t_process))

print("Done!")

# End of Code
