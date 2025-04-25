# Raspberry-Pi-Setup
This repository contains the setup procedure of ROS2 on the Raspberry Pi


## OS Installation

- **OS**: Ubuntu MATE 22.04 (ARM64)  
- **Download**: [Ubuntu MATE 22.04 ARM64 (.img.xz 1.8 GB)](https://releases.ubuntu-mate.org/22.04/arm64/)

---

## Essential Software Installation

### Git
```bash
sudo apt install git
```

### OpenSSH Server
```bash
sudo apt install openssh-server
```

---

### ROS 2 Humble Installation

- **Reference**: [ROS 2 Humble Official Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

### Xacro Installation
```bash
sudo apt install ros-humble-xacro
```

---

### ros2_control and ros2_controllers Installation
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-hardware-interface
```

---

### Custom Hardware Message Support
```bash
sudo apt install ros-humble-controller-manager ros-humble-control-msgs
```

---

### 🎮 Wireless Controller Setup

```bash
sudo apt install joystick jstest-gtk evtest
```

- **Reference**: [Easy wireless control for your homemade robot!](https://youtu.be/F5XlNiCKbrY?si=w5_Fh4ZzALmVeLq5)

---

## Network Configuration (Netplan)

Create or modify a file in `/etc/netplan/` (e.g. `01-netcfg.yaml`) with the following content:

```yaml
network:
  version: 2
  renderer: NetworkManager

  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.201.127/24]  # Static IP for laptop or Raspberry Pi
      gateway4: 192.168.201.105        # IP of host/router/hotspot
      nameservers:
        addresses: [192.168.201.105]
      access-points:
        "Nikhil's M55":
          password: "Nikhil2000"
```

Apply configuration:
```bash
sudo netplan apply
```

---

## Remote Development

### VS Code SSH Setup

- Install the **Remote - SSH** extension by Microsoft in VS Code.

---

## Issues

### PlatformIO Python Interpreter Fix

- [Install PlatformIO-compatible Python](https://docs.platformio.org/en/latest/faq/install-python.html)
