# Command: mkfifo

The `mkfifo` command can be used to create *fifo* files, also called *named pipes*. They can be used for Inter-Process Communication (IPC).

### An example

Consider that we have two processes: a sender and a reciver. We can share data between them using a fifo.

Create a named fifo:

```shell
mkfifo channel.fifo
```

Now we define the two process in two different files:

```python
# file sender.py
with open("channel.fifo", "w") as f:
	f.write("Important message.")
```

```python
# file receiver.py
with open("channel.fifo", "r") as f:
	msg = f.readline()
	print("Message:", msg)
```

If you run `python receiver.py` you will see that the program will stop: it will wait for a message. By also running `python sender.py`, the first program will continue and `Message: Important message.` will be printed out.

**Note** that the open call is blocking! It is blocking both in reading and writing mode. This can be useful for synchronization.