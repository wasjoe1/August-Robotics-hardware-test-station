# augustbot-tools

A collection of debug tools primarily written in Python and Bash.

This repository is supposed to be put in `~/catkin_ws/src/` so that
ROS can index and search the sub-folders for scripts.

Be forewarned the dependencies are not configured here,
you will have to get all ROS-related stuff pre-configured,
including (1) ROS workspace (2) custom message types.

Further system-wide dependencies will be described in the
respective README residing in the `*_tools` folders.

## Shortcuts
- [bb_tools/README.md](bb_tools/README.md)
- [sw_tools/README.md](sw_tools/README.md)

## Some useful Linux setup
```bash
# common packages on ubuntu
sudo apt install nano vim htop tree ncdu sshpass curl wget

# in ~/.bashrc
alias l="ls -l"
alias ll="ls -al"
export HISTSIZE=-1
export HISTFILESIZE=-1

# fuzzy search tools
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install # apt install fzf for Ubuntu >=19.10
```

## About extracting files from other machines

### Easy (GUI) way
- Open your favorite file manager GUI (e.g. `thunar`, `nautilus`, or simply type `gio open .`)
- Press `Ctrl-L` to jump to the address bar
- Type in your destination `sftp://augudisi@192.168.43.99`, and answer the password prompt
- Navigate and use like a local directory

### Hard but efficient way
```bash
# basic
rsync -avP <from-src> <to-dest>
# dry runs
rsync -avP <from-src> <to-dest> -n
# sync and remove extras
rsync -avP <from-src> <to-dest> --delete
# example: dry run to get files from robot(diego) to local machine
rsync -avP augdisi@192.168.43.99:~/catkin_ws/src/swivelbot ./swivelbot --delete -n
```

## Bash often used features/syntax (best practices & boilerplatess
- https://kvz.io/bash-best-practices.html
- https://linuxcommand.org/lc3_wss0150.php
- https://mywiki.wooledge.org/BashPitfalls
- https://wiki.bash-hackers.org/
- https://zwischenzugs.com/2018/01/21/ten-more-things-i-wish-id-known-about-bash/
- http://www.shelldorado.com/goodcoding/cmdargs.html
- https://www.davidpashley.com/articles/writing-robust-shell-scripts/
- http://linux.vbird.org/linux_basic/0330regularex.php
- https://regexlearn.com/