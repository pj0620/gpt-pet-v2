#!/bin/bash
# Source this (bash or zsh) on the Mac dev machine to join the robot's ROS 2 graph:
#   source bin/mac_env.sh
# Afterwards ros2 CLI, rviz2, foxglove_bridge, teleop and bin/motors/*.sh work from this machine.
#
# What it does and why:
#  - activates the RoboStack ROS 2 Humble env (mamba env "ros_env")
#  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp: the robot runs Fast DDS, and a CycloneDDS peer (the
#    ~/.zshrc default on this Mac) never discovers it
#  - ROS_DOMAIN_ID=0, the robot's domain
#  - multicast discovery does not cross this WiFi network (and does not even work between two
#    processes on this Mac), so a Fast DDS profile listing localhost, the robot and the server as
#    unicast initial peers is generated (hostnames resolved via mDNS each time)
#  - restarts the ros2 CLI daemon so it does not keep a stale, empty view of the graph
# Overrides: GPTPET_PEERS="192.168.1.27,192.168.1.50" (IPs, skips resolution),
#            GPTPET_PEER_HOSTS="gptpetclient.local pj-ubuntu.local", GPTPET_ROS_ENV=<mamba env path>

GPTPET_ROS_ENV="${GPTPET_ROS_ENV:-$HOME/.local/share/mamba/envs/ros_env}"
GPTPET_PEER_HOSTS="${GPTPET_PEER_HOSTS:-gptpetclient.local pj-ubuntu.local}"

if [ -n "$ZSH_VERSION" ]; then
  _gptpet_dir="$(cd "$(dirname "${(%):-%N}")" && pwd)"
  source "$GPTPET_ROS_ENV/setup.zsh"
else
  _gptpet_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  source "$GPTPET_ROS_ENV/setup.bash"
fi
export GPTPET_REPO="$(cd "$_gptpet_dir/.." && pwd)"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI

# hostname -> IPv4 via mDNS, empty if not on the network
_gptpet_resolve() {
  local ip
  ip=$(dscacheutil -q host -a name "$1" 2>/dev/null | awk '/^ip_address/{print $2; exit}')
  if [ -z "$ip" ]; then
    ip=$(ping -c1 -t1 "$1" 2>/dev/null | sed -n 's/.*(\([0-9.]*\)).*/\1/p' | head -1)
  fi
  echo "$ip"
}

# 127.0.0.1 is always a peer: without it rviz2, foxglove_bridge, teleop and the ros2 CLI on this
# Mac would not discover each other.
_gptpet_peers="127.0.0.1"
_gptpet_found=""
_gptpet_missing=""
if [ -n "$GPTPET_PEERS" ]; then
  _gptpet_peers="$_gptpet_peers $(echo "$GPTPET_PEERS" | tr ',' ' ')"
  _gptpet_found=" $(echo "$GPTPET_PEERS" | tr ',' ' ')"
else
  for _h in $(echo "$GPTPET_PEER_HOSTS"); do
    _ip=$(_gptpet_resolve "$_h")
    if [ -n "$_ip" ]; then
      _gptpet_peers="$_gptpet_peers $_ip"
      _gptpet_found="$_gptpet_found $_h=$_ip"
    else
      _gptpet_missing="$_gptpet_missing $_h"
    fi
  done
fi

# Fast DDS participant profile: unicast initial peers. One locator per metatraffic unicast port
# (7410 + 2 * participant_id, participants 0..9) so every ROS process on a peer host is reached.
export FASTRTPS_DEFAULT_PROFILES_FILE="${TMPDIR:-/tmp}/gptpet_fastdds_peers.xml"
_gptpet_xml='<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="gptpet_mac" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>'
for _ip in $(echo "$_gptpet_peers"); do
  for _pid in 0 1 2 3 4 5 6 7 8 9; do
    _gptpet_xml="$_gptpet_xml
          <locator><udpv4><address>$_ip</address><port>$((7410 + 2 * _pid))</port></udpv4></locator>"
  done
done
_gptpet_xml="$_gptpet_xml
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>"

# Only when the peer list changed: write the profile and restart the ros2 CLI daemon, which
# otherwise keeps a stale (possibly empty) view of the graph. Every bin/*.sh script sources this
# file, so an unconditional restart here would kill the daemon under other running CLI commands.
if [ "$_gptpet_xml" != "$(cat "$FASTRTPS_DEFAULT_PROFILES_FILE" 2>/dev/null)" ]; then
  printf '%s\n' "$_gptpet_xml" > "$FASTRTPS_DEFAULT_PROFILES_FILE"
  ros2 daemon stop >/dev/null 2>&1
  # start it now so it has discovered the robot by the time the first ros2 command runs; without
  # this, "ros2 topic echo <topic>" right after sourcing fails with "could not determine the type"
  ros2 daemon start >/dev/null 2>&1
fi

echo "gptpet: ROS 2 $ROS_DISTRO, $RMW_IMPLEMENTATION, domain $ROS_DOMAIN_ID, DDS peers:${_gptpet_found:- none}${_gptpet_missing:+ (not on network:$_gptpet_missing)}"
unset _gptpet_dir _gptpet_peers _gptpet_found _gptpet_missing _gptpet_xml _h _ip _pid
