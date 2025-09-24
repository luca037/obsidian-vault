# Define train-validation-test set

This is useful for multi-classification tasks. Consider that you have a directory structured as follows:

```
.
├── test
│   ├── class1 
│   ├── class2
│   ├── class3
│   └── ...
└── train
    ├── class1 
    ├── class2
    ├── class3
    └── ...
```

Then creating a train-validation-test split is easy:

 ```python
def create_datasets():
    train_ds, val_ds = image_dataset_from_directory(
        directory=TRAIN_DIR,
        labels='inferred',
        label_mode='categorical',
        batch_size=BATCH_SIZE,
        image_size=(IMG_HEIGHT, IMG_WIDTH),
        validation_split=0.2,
        seed=123,
        subset="both"
    )

    test_ds = image_dataset_from_directory(
        directory=TEST_DIR,
        labels='inferred',
        label_mode='categorical',
        batch_size=BATCH_SIZE,
        image_size=(IMG_HEIGHT, IMG_WIDTH)
    )

    return train_ds, val_ds, test_ds
 ```

# Evaluate a model

Consider a (multi-)classification task. Define a function that plots the accuracy and loss over epochs:

```python
import matplotlib.pyplot as plt

def evalutate_model(model, test_ds, history):
    # Evaluate the model on the test dataset.
    val_loss, val_accuracy = model.evaluate(test_ds)
    print(f'Test Accuracy: {val_accuracy:.4f}')
    print(f'Test Loss: {val_loss:.4f}')
    
    # Plot training & validation accuracy values.
    plt.figure(figsize=(14, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')
    
    # Plot training & validation loss values
    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')
    
    plt.show()

```

We can also define a function that plots the confusion matrix:

```python
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
import seaborn as sns

def plot_confusion_matrix(model, test_ds, class_names):
	# Compute predictions on test set.
    Y_pred = model.predict(test_ds)
    y_pred = np.argmax(Y_pred, axis=1)
	
	# Define true labels.
    y_true = np.concatenate([
	    y.numpy() 
	    for x, y in test_ds
	], axis=0)
    y_true = np.argmax(y_true, axis=1)

	# Copute confusion matrix and plot it.
    cm = confusion_matrix(y_true, y_pred)
	
    plt.figure(figsize=(8,6))
    sns.heatmap(
	    cm, annot=True, fmt="d", cmap="Blues",
	    xticklabels=class_names, 
		yticklabels=class_names
	)
    plt.xlabel("Predicted")
    plt.ylabel("True")
    plt.title("Confusion Matrix")
    plt.show()
```

Then you just need to train a model and call the two functions:

```python
history = model.fit(train_ds, EPOCHS, val_ds)
evalutate_model(model, test_ds, history)
plot_confusion_matrix(model, test_ds, CLASS_NAMES)
```

# Transfer Learning & Fine-tuning

Consider that we want to fine-tune a pre-trained `EfficientNetB0`, which is a CNN. Consider a multi-classification task.

We can define a function that returns the model:

```python
def build_model():
	# Define the base model.
    base_model = EfficientNetB0(
        include_top=False, # Remove output layer
        weights="imagenet",
        input_shape=(height, width, channels)
    )
	
	# Freeze base model.
    base_model.trainable = False
	
	# Define the full model: add output layer.
    inputs = base_model(x)
    x = GlobalAveragePooling2D()(x) # (i.e. flatten)
    outputs = Dense(num_classes, activation='softmax')(x)
	
	return Model(inputs, outputs)
```

Now we can apply transfer learning.

```python
# Create datasets.
train, val, test = create_datasets()

# Define the model.
model = build_model()

# Compile model.
model.compile(
    optimizer='adam',
    loss='categorical_crossentropy',
    metrics=['accuracy']
)

# Train only last layer.
model.fit(
    train,
    epochs=EPOCHS_PRETRAIN,
    validation_data=val
)
```

Finally we can fine tune the model for a few more epochs. In this case we will unfreeze all the layers. **Note:** batch normalization layer must stay freeze.

```python
# Retreive base model.
base_model = model.get_layer("efficientnet-b0")
for layer in base_model.layers:
	if not isinstance(layer, BatchNormalization):
		layer.trainable = True

# Compile the model with a lower learning rate.
model.compile(
	optimizer=Adam(learning_rate=0.0001),
	loss='categorical_crossentropy',
	metrics=['accuracy']
)

# Fine-tune model.
model.fit(
    train,
    epochs=EPOCHS_FINETUNE,
    validation_data=val
)
```


To get the name of the base model you can simply plot a summary of your model with `model.summary()`.

**Resources:**

- [Keras - Transfer learning & fine-tuning](https://keras.io/guides/transfer_learning/)