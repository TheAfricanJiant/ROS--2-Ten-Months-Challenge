# 🔄 Workflow between the Raspberry Pi and the Windows machine

[← Back to repository index](../README.md)

ROS 2 packages get written and built on the **Raspberry Pi**, because that is
where ROS 2 actually runs. The repository lives on **Windows**, which is what
talks to GitHub. This is how work moves between them without copying folders
about and without the Pi ever needing GitHub credentials.

```
                    GitHub
                       ↑
                       │ push
                       │
    Windows ───────────┘
       │
       │ git fetch over LAN
       ↓
  Raspberry Pi
       │
       └── creates the ROS 2 package
```

The Pi is the one **producing** commits; Windows **pulls them in over the LAN**
and is the only machine that talks to GitHub.

---

## The setup

On the Pi, your existing GitHub clone stays the working repository. Nothing
special is needed there — no credentials, no remote, no push access.

On Windows, add the Pi as a **temporary Git remote**, once:

```bash
cd C:\path\to\your-repo
git remote add raspberry pi@192.168.1.50:/home/pi/your-repo
```

Replace the address with your Pi's. Find it on the Pi with `hostname -I`, or
use `raspberrypi.local` if mDNS works on your network. This is a plain SSH
remote, so Git goes over the same SSH you already use.

---

## The cycle

### 1. On the Pi — do the work and commit

```bash
cd ~/your-repo
ros2 pkg create --build-type ament_python my_package
git add .
git commit -m "Add ROS 2 package"
```

**Do not push.** The Pi has nothing to push to, and does not need it.

### 2. On Windows — fetch and merge over the LAN

```bash
git fetch raspberry
git log raspberry/main          # see what the Pi has
git merge raspberry/main        # bring it into your branch
```

### 3. On Windows — push to GitHub

```bash
git push origin main
```

That is the whole loop. The Pi's work arrives as **proper Git history**, with
its own commits and messages, not as a folder someone copied.

---

## Use `fetch` + `merge`, not `pull`

`git pull` is `fetch` followed by `merge` in one step. Both do the same thing
here, but writing them separately makes it explicit that you are **importing
work from another machine**, and it gives you a chance to look at
`git log raspberry/main` before anything touches your branch.

Worth the extra keystrokes when the two machines can drift apart.

---

## Going the other way

When Windows has commits the Pi needs — a config file you edited, or something
you pulled from GitHub — the Pi fetches from GitHub as usual:

```bash
cd ~/your-repo
git pull origin main
```

The Pi only ever **reads** from GitHub. It never pushes.

---

## Practical notes

**Set up SSH keys** so `git fetch raspberry` does not ask for a password every
time. From Windows:

```bash
ssh-keygen -t ed25519            # if you do not already have a key
ssh-copy-id pi@192.168.1.50      # or paste it into ~/.ssh/authorized_keys
```

**If the Pi's IP moves**, update the remote rather than adding a second one:

```bash
git remote set-url raspberry pi@192.168.1.60:/home/pi/your-repo
```

**Check what remotes you have** at any time:

```bash
git remote -v
```

**Merge conflicts** happen when both machines changed the same file. Resolve
them on Windows, where the merge is running — the Pi is unaffected.

**Line endings.** The Pi writes LF, Windows may write CRLF. If diffs show whole
files as changed, set this once on Windows:

```bash
git config core.autocrlf input
```

**The Pi must be reachable** for `git fetch raspberry` to work — same LAN, or a
VPN. If it is not, that command is the only thing that fails; everything else
still works normally.

---

## Why not just push from the Pi?

You could, but then the Pi needs GitHub credentials stored on it — a token or
an SSH key with write access to your repository. On a robot that may be shared,
carried around, or reflashed, that is a credential you have to think about.

Fetching from the Pi keeps all GitHub access on the one machine you control,
and the Pi stays a disposable working copy.
