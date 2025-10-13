Here I just want to report all the basic stuff that I usually forget how
to do and I always find myself searching online over and over for the same questions.

## Create venv

Useful if you want to manage python packages using pip instead of your default package manager.

You can create `.python-venv` in your home folder and then use the `venv` module to create the virtual environment:

```shell
python -m venv $HOME/.python-venv/
```

Then you can activate the environment using:

```shell
source $HOME/.python-venv/bin/activate
```

To make things easy you can crate an alias in your `.bashrc` or `.zshrc` or whatever. I added the following line:

```shell
alias python-venv = "source $HOME/.python-venv/bin/activate"
```

## OS library

Some basic operations to manage files and directories.

```python
import os

dir_path = "your_dir_path"

# Check if a directory exists, if not => create it.
if not os.path.isdir(dir_path):
	os.makedirs(dir_path)

# Get a list of all file names inside a directory.
dir_filenames = os.listdir(dir_path)

# Get full path name of file inside the directory.
filename = dir_filenames[0]
file_path = os.path.join(dir_path, filename)

# Is this a file or a dir?
if os.path.isfile(filename):
	print("Is a file!")
```

## Matplotlib

An extremely simple plot.

```python
import matplotlib.pyplot as plt

x = [i for i in range(10)]
y = [i**2 for i in x]

plt.plot(x, y)
plt.show()
```