# AltServer - Wifi refresh

Small tutorial on how to refresh apps in alt-store through wifi, using a raspberry pi.

First of all you need to activate Wifi-sync option on your iDevice. To do that you can follow [this](https://support.apple.com/guide/itunes/wi-fi-syncing-itns3751d862/windows) link.

### Pre-requisites

Required packages:
```shell
sudo apt install -y \
  libavahi-compat-libdnssd-dev \
  usbmuxd \
  ninja-build \
  ldc \
  libplist-dev \
  libimobiledevice-dev \
  libgtk-3-0 \
  dub \
  avahi-utils \
  openssl
```

Build from source the following packages:
1. [libimobiledevice-glue](https://github.com/libimobiledevice/libimobiledevice-glue#debian--ubuntu-linux)
2. [libimobiledevice](https://github.com/libimobiledevice/libimobiledevice#debian--ubuntu-linux)
This installs executables such as `idevicepair` and `ideviceinfo`. We can use those later to verify if our wireless connection to the idevice is working properly.

Install [rustup](https://rustup.rs/), then run:

```shell
rustup toolchain install stable
rustup default stable
```

Edit file `/lib/systemd/system/usbmuxd.service` and add the following:

```
[Install]
WantedBy=multi-user.target
```

Then start and enable the service with:

```shell
sudo systemctl enable --now avahi-daemon.service
sudo systemctl enable --now usbmuxd
```

Download `AltServer-aarch64` binary from the following [link](https://github.com/NyaMisty/AltServer-Linux/releases) and the binary `aarch64-linux-netmuxd` from this [link](https://github.com/jkcoxson/netmuxd/releases). From command line you can use:

```shell
wget github.com/<user>/<repo>/releases/download/vX.Y.Z/<file>
```

Now you can download Docker by following [this](https://pimylifeup.com/raspberry-pi-docker/) link.

### Make sure everything works as expected

With docker installed, you can now run the following command to start the `anisette` container:

```shell
docker run -d -v lib_cache:/opt/lib/ --restart=always -p 6969:6969 --name anisette dadoum/anisette-server:latest
```

If you have already activate Wifi-sync, than you can run

```shell
avahi-browse -a
```
In the output  you should see your iDevice.

Now you can connect it to the pi via USB and tap trust on the iDevice to **trust** the pi. Verify that the pairing was successful by running `idevicepair validate`.

 Run sudo `./aarch64-linux-netmuxd` to start the netmuxd proxy. The output should be something like:
 
```shell
Starting netmuxd
Starting mDNS discovery for _apple-mobdev2._tcp.local with mdns
Listening on /var/run/usbmuxd
Adding device 00008110-0123A456B789D012 
```

(The device number can be different!).

Now you can run `AltServer` with:

```shell
ALTSERVER_ANISETTE_SERVER=http://127.0.0.1:6969 ./AltServer-aarch64
```

If you have already installed `AltStore`, then you should be able to refresh through Wifi.

### Auto-start

First you need to modify your `sudoers` file (with `sudo visudo`) by adding:

```shell
<your usr hame> ALL=(ALL) NOPASSWD: /usr/bin/aarch64-linux-netmuxd
```

Finally you need to modify your `.bashrc` by adding the following lines:

```shell
if pgrep -f "aarch64-linux-netmuxd" > /dev/null; then
	echo "aarch64-linux-netmuxd is already running."
else
	sudo aarch64-linux-netmuxd \
	> $HOME/Logs/netmuxd.txt 2>&1 &
fi

if pgrep -f "anisette-server-aarch64" > /dev/null; then
	echo "anisette-server-aarch64 is already running."
else
	anisette-server-aarch64 -n 127.0.0.1 -p 6969 \
	> $HOME/Logs/anisette.txt 2>&1 &
fi

if pgrep -f "AltServer-aarch64" > /dev/null; then
	echo "AltServer-aarch64 is already running."
else
	ALTSERVER_ANISETTE_SERVER=http://127.0.0.1:6969 \
	AltServer-aarch64 \
	> $HOME/Logs/altserver.txt 2>&1 &
fi
```

To apply the changes you can `source .bashrc` and everything should work.
### References links:
- [Main reference](https://gist.github.com/jschiefner/95a22d7f4803e7ad32a95b0f3aa655dc)
- Original reddit posts:
	- [Post 1](https://www.reddit.com/r/jailbreak/comments/wa4z2z/tutorial_altstore_wifi_refresh_on_raspberry_pi/)
	- [Post 2](https://www.reddit.com/r/jailbreak/comments/wa4z2z/tutorial_altstore_wifi_refresh_on_raspberry_pi/)
