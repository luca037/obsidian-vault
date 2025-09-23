# Introduction to Keras

# Define train-validation-test set

This is useful for multi-classification tasks. This can be used if the dataset is structured in a certain way.

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


## Fine-tune a pre-trained model

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

