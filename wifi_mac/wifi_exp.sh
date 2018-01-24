#!/bin/bash
# Configure WiFi transceivers
# Usage:
#   ./wifi_exp.sh   -- Configure 1 USRP
#   ./wifi_exp.sh 1 -- same as above
#   ./wifi_exp.sh 2 -- Configure 2 USRPs

# kill living processes
ps -ef | grep -e ul_buffer.py -e phy_wifi.py | grep -v grep | awk '{print $2}' | xargs kill

set -e  # terminate installation upon error occurs

n_usrp=1

if [ $# == 1 ] && [ $1 == 2 ]; then
    n_usrp=2
fi

echo "$n_usrp USRP(s) used"

# PHY parameters
PHYport=(8013 8014)     # Socket No. of PHY
PHYRXport=(8513 8514)   # Socket No. of PHY RX
localNodeId=(1 2)       # Local node
destNodeId=(2 1)        # Destination node
usrp_addr=("" "192.168.10.2")
tx_gain=(15 23.5)
rx_gain=(5.7 5.7)

# Buffer parameters
MACport=(8001 8002)     # Socket No. of upper layer (buffer)
n_pkt=(100 0)         # No. of pkt generated
t_interval=(0.02 0.02)  # Interval of pkt generation (s)

# MAC parameters
rate_control=(none none)    # rate adaptation approach
                            # "none"
                            # "aarf"
                            # "minstrel"
encoding=(0 0)      # initial data rate
beta=15000          # scaling factor of timing
                    # 1 USRP: 1000
                    # 2 USRPs: 15000
retx_max=(0 0)      # Maximum No. of retries

# Sanity check
if [ "${rate_control[0]}" != none ] && [ "${rate_control[0]}" != "aarf" ] && [ "${rate_control[0]}" != "minstrel" ]
then
    echo "Wrong rate adaptation parameter for USRP Node ${localNodeId[0]}: ${rate_control[0]}"
    echo "Expected: none, aarf, minstrel"
    exit
fi

if [ ${localNodeId[0]} == ${destNodeId[0]} ]; then
    echo "Node ${localNodeId[0]} can\'t transmit to itself"
    exit
fi

if [ $n_usrp == 2 ]; then

    if [ ${localNodeId[1]} == ${destNodeId[1]} ]; then
        echo "Node ${localNodeId[1]} can\'t transmit to itself"
        exit
    fi

    if [ "${rate_control[1]}" != none ] && [ "${rate_control[1]}" != "aarf" ] && [ "${rate_control[1]}" != "minstrel" ]
    then
        echo "Wrong rate adaptation parameter for USRP ${localNodeId[1]}: ${rate_control[1]}"
        echo "Expected: none, aarf, minstrel"
        exit
    fi

    if [ ${PHYport[0]} == ${PHYport[1]} ]; then
        echo "PHYports are the same!"
        exit
    fi

    if [ ${PHYRXport[0]} == ${PHYRXport[1]} ]; then
        echo "PHYRXport are the same!"
        exit
    fi

    if [ ${MACport[0]} == ${MACport[1]} ]; then
        echo "MACport are the same!"
        exit
    fi

    if [ ${localNodeId[0]} == ${localNodeId[1]} ]; then
        echo "localNodeId are the same!"
        exit
    fi

    if [ "${usrp_addr[0]}" == "${usrp_addr[1]}" ]; then
        echo "usrp_addr are the same!"
        exit
    fi
fi

# Run commands
dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $dir
echo "Entering $PWD"

prefix="python "

##########################################
# Configure the second USRP (optional)
##########################################

# Assemble commands
if [ $n_usrp == 2 ]; then
    usrp_no=1
    buf_cmd="ul_buffer.py --MACport=${MACport[$usrp_no]}"
    tra_cmd="ul_traffic.py --MACport=${MACport[$usrp_no]} -n ${n_pkt[$usrp_no]} -t ${t_interval[$usrp_no]}"
    phy_cmd="phy_wifi.py -n ${localNodeId[$usrp_no]} --PHYport=${PHYport[$usrp_no]} --PHYRXport=${PHYRXport[$usrp_no]}\
    -G ${tx_gain[$usrp_no]} -g ${rx_gain[$usrp_no]}"
    if [ "${usrp_addr[$usrp_no]}" != "" ]; then
        phy_cmd="$phy_cmd -a ${usrp_addr[$usrp_no]}"
    fi
    mac_cmd="mac_wifi.py --MACport=${MACport[$usrp_no]} --PHYport=${PHYport[$usrp_no]} --encoding=${encoding[$usrp_no]}\
     --beta=$beta -r ${retx_max[$usrp_no]} -R ${rate_control[$usrp_no]} --dest_node=${destNodeId[$usrp_no]}"

    echo "======= USRP Node ${localNodeId[$usrp_no]} ========"
    echo "CMD 1: $buf_cmd"
    echo "CMD 2: $tra_cmd"
    echo "CMD 3: $phy_cmd"
    echo "CMD 4: $mac_cmd"
    echo "============================"

    # Run commands
    echo "[${localNodeId[$usrp_no]}] Start PHY"
    ($prefix$phy_cmd) &
    sleep 5

    echo "[${localNodeId[$usrp_no]}] Start buffer"
    ($prefix$buf_cmd) &
    sleep 5

    # echo "[${localNodeId[$usrp_no]}] Generate traffic"
    # $prefix$tra_cmd
    # echo "[${localNodeId[$usrp_no]}] Traffic generated"
    # sleep 5

    echo "[${localNodeId[$usrp_no]}] Start MAC"
    ($prefix$mac_cmd) &
fi

##########################################
# Configure the first USRP
##########################################

# Assemble commands
usrp_no=0
buf_cmd="ul_buffer.py --MACport=${MACport[$usrp_no]}"
tra_cmd="ul_traffic.py --MACport=${MACport[$usrp_no]} -n ${n_pkt[$usrp_no]} -t ${t_interval[$usrp_no]}"
phy_cmd="phy_wifi.py -n ${localNodeId[$usrp_no]} --PHYport=${PHYport[$usrp_no]} --PHYRXport=${PHYRXport[$usrp_no]}\
    -G ${tx_gain[$usrp_no]} -g ${rx_gain[$usrp_no]}"
if [ "${usrp_addr[$usrp_no]}" != "" ]; then
    phy_cmd="$phy_cmd -a ${usrp_addr[$usrp_no]}"
fi
mac_cmd="mac_wifi.py --MACport=${MACport[$usrp_no]} --PHYport=${PHYport[$usrp_no]} --encoding=${encoding[$usrp_no]}\
 --beta=$beta -r ${retx_max[$usrp_no]} -R ${rate_control[$usrp_no]} --dest_node=${destNodeId[$usrp_no]}"

echo "======= USRP Node ${localNodeId[$usrp_no]} ========"
echo "CMD 1: $buf_cmd"
echo "CMD 2: $tra_cmd"
echo "CMD 3: $phy_cmd"
echo "CMD 4: $mac_cmd"
echo "============================"

echo "[${localNodeId[$usrp_no]}] Start PHY"
($prefix$phy_cmd) &
sleep 5

echo "[${localNodeId[$usrp_no]}] Start buffer"
($prefix$buf_cmd) &
sleep 5

echo "[${localNodeId[$usrp_no]}] Generate traffic"
$prefix$tra_cmd
echo "[${localNodeId[$usrp_no]}] Traffic generated"
sleep 5

echo "[${localNodeId[$usrp_no]}] Start MAC"
($prefix$mac_cmd) &

cd --

trap 'kill $BGPID; exit' SIGINT
sleep 1024 &    # background command
BGPID=$!
sleep 1024      # foreground command of the script
