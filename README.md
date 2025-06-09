# Raspberry Pi GPIO Package

## ⚠️ Hardware Requirement

> **Note:** This package is designed **exclusively** for **Raspberry Pi hardware**.

---

## ⚙️ WiringPi Dependency

You'll need to build the **WiringPi** library from source. It is **not** available via `apt` anymore, so ensure it's correctly installed before proceeding.

---

# Installation Instructions for WiringPi

```sh
# fetch the source
cd
sudo apt install git
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi

# build the package
./build debian
cd debian-template/wiringpi_3.14_amd64.deb 

# install it
sudo apt install ./wiringpi_3.14_amd64.deb
```
---

## 🔧 I2C Access Permissions

To allow user-space access to I2C (via `/dev/gpiomem`), run the following commands:

```bash
sudo chmod g+rw /dev/gpiomem
sudo chown root.gpio /dev/gpiomem
```

