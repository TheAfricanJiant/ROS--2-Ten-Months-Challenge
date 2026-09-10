# Build Process

This project uses **PlatformIO** and **micro-ROS**.

> **Important:** For Windows, build the project inside **WSL (Ubuntu)** rather than using native Windows PlatformIO.

---

# 1. Windows

## Prerequisites

Install:

- Git
- VS Code
- WSL2
- Ubuntu (WSL)

### Install WSL

Open **PowerShell as Administrator**:

```powershell
wsl --install
```

Restart Windows if required.

Open Ubuntu after installation:

```bash
wsl
```

---

## Clone the Repository

Inside Ubuntu/WSL:

```bash
cd ~
git clone <REPOSITORY_URL>
cd <REPOSITORY_FOLDER>
```

If the repository already exists:

```bash
cd <REPOSITORY_FOLDER>
git pull
```

---

## Install PlatformIO

Install PlatformIO CLI inside WSL:

```bash
python3 -m pip install --user platformio
```

Verify:

```bash
pio --version
```

If `pio` is not found:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

Then:

```bash
pio --version
```

---

## Build

From the project directory:

```bash
pio run
```

To clean the previous build:

```bash
pio run -t clean
```

Then build again:

```bash
pio run
```

### Output Binary Location
The compiled file is created at:
```text
.pio/build/pico/firmware.uf2
```
To commit it to Git for drag-and-drop flashing:
```bash
cp .pio/build/pico/firmware.uf2 ./firmware.uf2
```

---

## Build From VS Code

Open the project from WSL:

```bash
code .
```

Make sure VS Code shows the **WSL** environment.

Then use the PlatformIO build command:

```text
PlatformIO → Build
```

Do **not** open the project as a normal Windows PlatformIO project.

---

## Git Workflow

After making changes:

```bash
git status
git add .
git commit -m "Update project"
git push
```

On another machine:

```bash
git pull
```

The GitHub repository is used for **source-code synchronization**.

The WSL environment is used for the **micro-ROS/PlatformIO build**.

---

# 2. Linux / macOS

## Prerequisites

Install:

- Git
- Python 3
- PlatformIO

---

## Clone the Repository

```bash
git clone <REPOSITORY_URL>
cd <REPOSITORY_FOLDER>
```

If the repository already exists:

```bash
cd <REPOSITORY_FOLDER>
git pull
```

---

## Install PlatformIO

```bash
python3 -m pip install --user platformio
```

Verify:

```bash
pio --version
```

If `pio` is not found:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

Then:

```bash
pio --version
```

---

## Build

From the project directory:

```bash
pio run
```

To clean the previous build:

```bash
pio run -t clean
```

Build again:

```bash
pio run
```

### Output Binary Location
The compiled file is created at:
```text
.pio/build/pico/firmware.uf2
```
To commit it to Git for drag-and-drop flashing:
```bash
cp .pio/build/pico/firmware.uf2 ./firmware.uf2
```

---

## Git Workflow

Push changes:

```bash
git status
git add .
git commit -m "Update project"
git push
```

Pull changes:

```bash
git pull
```

---

# Quick Reference

| Operating System | Build Environment |
|---|---|
| Windows | **WSL / Ubuntu** |
| Linux | Native |
| macOS | Native |

Build command:

```bash
pio run
```

Clean command:

```bash
pio run -t clean
```

Update repository:

```bash
git pull
```

Push changes:

```bash
git add .
git commit -m "Update project"
git push
```