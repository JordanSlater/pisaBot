Raspberry pi Setup:
1. Install Ubuntu Server on SD card. Configure the user as pisaBot and the ssh keys.
2. Configure the network:
    1.  Run this for the sd card `wpa_passphrase "Other network name" password | sudo tee -a mnt/sd_card/etc/netplan/50-cloud-init.yaml`
    2. Then reformat the to match the above link. i.e.
        ```
        access-points:
        "SSID1":
            password: "pass1"
        "SSID2":
            password: "pass2"
        ```
3. Boot the pi up with the sd card and ssh in.
4. `sudo apt update`
5. Install ROS (ros-noetic-desktop)
    1. Configure your Ubuntu repositories:
        ```
        $ sudo add-apt-repository universe
        $ sudo add-apt-repository multiverse
        $ sudo add-apt-repository restricted
        ```
        To check that this worked:
        ```
        $ grep ^deb /etc/apt/sources.list
        ```
6. In a separate terminal clone jshen, and pisaBot. Use `git submodule update --init` to clone submodules after clone.
7. Add `jshen/src/machine/pisaBot.bash`:
    ```
    #!/bin/bash
    source $HOME/pisaBot/devel/setup.bash
    ```
7. Get MPU-6050 working:
    ```
    sudo apt-get install libi2c-dev i2c-tools libi2c0
    ```
8. Get gpio raspberry pi working:
    1. `sudo pip install RPi.GPIO`
    2. sudo adduser jordan gpio
9. `catkin_make` in piasBot.

