# RCETI
Robotic Control of Endotracheal Tube Insertion

## For Setup of Ubuntu Machine (For Connection with Raspberry Pi)

### Setting Up Static IP
- Open Settings > Network.
- Click on Wired Connection (the Ethernet interface).
- Click Settings (⚙️).
- Go to the IPv4 tab.
- Select Manual and set:
- Address: 192.168.2.1
- Netmask: 255.255.255.0
- Gateway: Leave blank
- Click Apply and disconnect/reconnect Ethernet.

### Setting Up ROS to use static IP
Enter the following commands into your terminal in Ubuntu 22.04

`echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc ` \
`echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`echo "export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`source ~/.bashrc` \

This should set up your ROS IP so that it can talk with the Raspberry Pi

