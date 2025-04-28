## Install

```bash
mkcert -install
mkcert -cert-file cert.pem -key-file key.pem 192.168.37.124 localhost 127.0.0.1

cd path/to/vuer
pip install --prefix=~/.local -e  '.[all]'

pip install zmq
```

```bash
colcon build
source install/setup.bash
ros2 run pico_driver monitor

# https://192.168.37.124:8012/?ws=wss://192.168.37.124:8012
```