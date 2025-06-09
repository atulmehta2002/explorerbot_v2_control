# Raspberry Pi GPIO Package

## ⚠️ Hardware Requirement

> **Note:** This package is designed **exclusively** for **Raspberry Pi hardware**.

---

## ⚙️ WiringPi Dependency

You'll need to build the **WiringPi** library from source. It is **not** available via `apt` anymore, so ensure it's correctly installed before proceeding.

---

## 🔧 I2C Access Permissions

To allow user-space access to I2C (via `/dev/gpiomem`), run the following commands:

```bash
sudo chmod g+rw /dev/gpiomem
sudo chown root.gpio /dev/gpiomem
```

