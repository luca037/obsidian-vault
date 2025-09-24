
# Plot Loss using a LossTracker

Consider an Autoencoder network. We will consider the example reported in a [tutorial](https://lightning.ai/docs/pytorch/stable/model/train_model_basic.html).

Some imports:

```python
import os
import torch
from torch import nn
import torch.nn.functional as F
from torchvision import transforms
from torchvision.datasets import MNIST
from torch.utils.data import DataLoader
import lightning as L
```

We define two networks using PyTorch `nn.Modules`:

```python
class Encoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.l1 = nn.Sequential(
	        nn.Linear(28 * 28, 64),
	        nn.ReLU(),
	        nn.Linear(64, 3)
	    )

    def forward(self, x):
        return self.l1(x)


class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.l1 = nn.Sequential(
	        nn.Linear(3, 64),
	        nn.ReLU(),
	        nn.Linear(64, 28 * 28)
	)

    def forward(self, x):
        return self.l1(x)
```

We define the AutoEncoder using PyTorch Lightning `L.LightningModule`:

```python
class LitAutoEncoder(L.LightningModule):
    def __init__(self, encoder, decoder):
        super().__init__()
        self.encoder = encoder
        self.decoder = decoder

    def training_step(self, batch, batch_idx):
        # training_step defines the train loop.
        x, _ = batch
        x = x.view(x.size(0), -1)
        z = self.encoder(x)
        x_hat = self.decoder(z)
        loss = F.mse_loss(x_hat, x)
		
		# Log loss.
		self.log('loss', loss)
		
        return loss

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(
	        self.parameters(), lr=1e-3
	    )
        return optimizer
```

Now we define the LossTracker class:

```python
class LossTracker(L.Callback):
    def __init__(self):
        self.train_loss = []

    def on_train_epoch_end(self, trainer, pl_module):
        loss = trainer.callback_metrics.get("loss").cpu()
        self.train_loss.append(loss)

    def plot(self):
        # Plot loss
        plt.plot(self.train_loss, label="Train loss")
        plt.xlabel("Epochs")
        plt.ylabel("Loss")
		plt.legend()
		plt.title("Training loss")
        plt.show()
```

Finally we can train and then plot the loss:

```python
# Define dataset.
dataset = MNIST(os.getcwd(), download=True, transform=transforms.ToTensor())
train_loader = DataLoader(dataset)

# Define model.
autoencoder = LitAutoEncoder(Encoder(), Decoder())

# Define LossTracker.
loss_tracker = LossTracker()

# Train model.
trainer = L.Trainer(callbacks=[loss_tracker])
trainer.fit(
	model=autoencoder,
	train_dataloaders=train_loader
)
loss_tracker.plot()
```
# Transfer Learning & Fine Tuning

Consider a simple case in which we want to apply transfer learning ad fine-tuning to a pre-trained model for multi-classification task. In this case we will use `ResNet-18` as a backbone.

First we define our class:

```python
class ResNetFineTuner(pl.LightningModule):
    def __init__(
	    self,
	    num_classes=10,
	    lr=1e-3,
	    freeze_backbone=True
	):
        super().__init__()
        self.save_hyperparameters()

        # Load pre-trained ResNet18.
        backbone = models.resnet18(pretrained=True)
		
		# Freeze the backbone, if necessary.
        if freeze_backbone:
            for param in backbone.parameters():
                param.requires_grad = False

        # Replace classifier head
        num_ftrs = backbone.fc.in_features
        backbone.fc = nn.Linear(num_ftrs, num_classes)
        self.model = backbone

        self.criterion = nn.CrossEntropyLoss()

    def forward(self, x):
        return self.model(x)

    def training_step(self, batch, batch_idx):
        x, y = batch
        logits = self(x)
        loss = self.criterion(logits, y)
        acc = (logits.argmax(dim=1) == y).float().mean()
        self.log('train_loss', loss)
        self.log('train_acc', acc, prog_bar=True)
        return loss

    def validation_step(self, batch, batch_idx):
        x, y = batch
        logits = self(x)
        val_loss = self.criterion(logits, y)
        acc = (logits.argmax(dim=1) == y).float().mean()
        self.log('val_loss', val_loss, prog_bar=True)
        self.log('val_acc', acc, prog_bar=True)

    def test_step(self, batch, batch_idx):
        x, y = batch
        logits = self(x)
        loss = self.criterion(logits, y)
        acc = (logits.argmax(dim=1) == y).float().mean()
        self.log('test_loss', loss)
        self.log('test_acc', acc)

    def configure_optimizers(self):
        return torch.optim.Adam(
	        self.parameters(),
	        lr=self.hparams.lr
	    )
```

Then we can simply create a model with:

```python
model = ResNetFineTuner()
```

All the layers of the backbone are already frozen, so we can train it:

```python
trainer = pl.Trainer(accelerator="auto", max_epochs=EPOCHS)
trainer.fit(model, train_dm)
```

Now we can fine tune the model, we just need to unfreeze some or all the layers:

```python
# Load best model found.
model = ResNetFineTuner.load_from_checkpoint(
	trainer.checkpoint_callback.best_model_path
)

# Unfreeze backbone.
for param in model.parameters():
	param.requires_grad = True

# Re-define trainer.
trainer = pl.Trainer(accelerator="auto", max_epochs=EPOCHS)

# Re-start training.
trainer.fit(model, train_dm)
```


