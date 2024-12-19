import matplotlib.pyplot as plt         # Library for visualization
import numpy as np                      # Library for numerical operations
import cv2                              # Library for image processing
import os                               # Library for file path manipulation
import PIL                              # Python Imaging Library (optional)
import tensorflow as tf                 # Library for deep learning
from tensorflow import keras            # Keras high-level API for models
from tensorflow.keras import layers     # Layers for building the CNN 
from tensorflow.keras.models import Sequential      # Sequential model for building CNNs
import pathlib                           # Used for path manipulation

# Define the data path
data = pathlib.Path('G:\Python Projects\FYP\dataset')

# Define a dictionary to map class names to labels
dataset = {
    'bb': list(data.glob('bigblue/*')),   # List of paths for big blue images
    'bg': list(data.glob('biggreen/*')),  # List of paths for big green images
    'sb': list(data.glob('smallblue/*')), # List of paths for small blue images
    'sg': list(data.glob('smallgreen/*')),# List of paths for small green images
}

# Define a dictionary to map class names to numerical labels
datalabel = {
    'bb': 0,  # Big blue box label
    'bg': 1,  # Big green box label
    'sb': 2,  # Small blue box label
    'sg': 3,  # Small green box label
}

# Initialize empty lists to store images and labels
x, y = [ ] , [ ]

# Loop through each class and its images
for data_name, images in dataset.items():
    for image_path in images:
        # Read the image using OpenCV
        img = cv2.imread(str(image_path))
        try:
            # Resize the image to a fixed size (360x180 in this case)
            resized_img = cv2.resize(img, (360, 180))
            x.append(resized_img)  # Add resized image to the data list
            # Get the label based on the class name
            label = datalabel[data_name]
            y.append(label)        # Add label to the label list
        except cv2.error as e:
            print(f"Error resizing image: {e}")  # Print error message if resizing fails

# Convert the lists to NumPy arrays for efficient processing
x = np.array(x)
y = np.array(y)

# Split data into training and testing sets using scikit-learn
from sklearn.model_selection import train_test_split

x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)
# Normalize the pixel values of the images (divide by 255)
x_train = x_train / 255
x_test = x_test / 255

# Define the CNN model architecture
model = Sequential([
    layers.Conv2D(16, 3, padding='same', activation='relu'),
    layers.MaxPooling2D(),
    layers.Conv2D(32, 3, padding='same', activation='relu'),
    layers.MaxPooling2D(),
    layers.Conv2D(64, 3, padding = 'same', activation = 'relu'),
    layers.MaxPooling2D(),
    layers.Conv2D(128, 3, padding = 'same', activation = 'relu'),
    layers.MaxPooling2D(),
    layers.Conv2D(256, 3, padding = 'same', activation = 'relu'),
    layers.MaxPooling2D(),
    layers.Conv2D(512, 3, padding = 'same', activation = 'relu'),
    layers.Dense(256, activation = 'relu'),
    layers.MaxPooling2D(),
    layers.Dropout(0.0),
    layers.Flatten(),
    layers.Dense(128, activation='relu'),
    layers.Dense(4)
])

# Compile the model with Adam optimizer, sparse categorical crossentropy loss, and accuracy metric
model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# Define callbacks for early stopping and learning rate reduction
callbacks = [
    EarlyStopping(
        monitor='val_loss',                  # Monitor validation loss
        patience=5,                    # Stop training if validation loss doesn't improve for 5 epochs
        restore_best_weights=True     # Restore weights from the best epoch
    ),
    ReduceLROnPlateau(
        monitor='val_loss',   # Monitor validation loss
        factor=0.2,                # Reduce learning rate by a factor of 0.2
        patience=3,               # Reduce learning rate if validation loss doesn't improve for 3 epochs
        min_lr=1e-6              # Minimum learning rate
    )]

# Train the model
history = model.fit(x_train, y_train,
                   validation_split=0.2,    # Split 20% of training data for validation
                   epochs=50,                   # Train for 50 epochs
                   batch_size=16,              # Process data in batches of 16 images
                   callbacks=callbacks)     # Use the defined callbacks

# Evaluate the model on the test data 
loss, accuracy = model.evaluate(x_test, y_test)

# Save the trained model 
model.save('cnn.model')
