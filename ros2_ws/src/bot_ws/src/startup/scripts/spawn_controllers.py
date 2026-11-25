#!/usr/bin/env python3
"""
Controller spawner with extended timeouts for non-RT systems (e.g. Jetson Nano).

The standard ros2_control spawner uses a hardcoded 10-second call timeout for
each service call (load / configure / switch). On the Jetson Nano without a
real-time kernel, CycloneDDS drops service responses under load — the controller
gets loaded but the response never arrives in time. This script uses a 120-second
timeout and treats 'already loaded/configured' as success so a previous timeout
retry doesn't cause a fatal exit.

Usage: spawn_controllers.py <controller1> [controller2] ...
"""
import sys
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ConfigureController,
    LoadController,
    SwitchController,
)

CALL_TIMEOUT = 120.0      # seconds to wait for each service response
DISCOVERY_TIMEOUT = 120.0 # seconds to wait for services to be discoverable


def call_with_timeout(node: Node, client, request, timeout: float = CALL_TIMEOUT):
    """Call a service and spin until done or timeout. Returns result or None."""
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        rclpy.spin_once(node, timeout_sec=0.5)
    return future.result()


def main() -> int:
    controllers = sys.argv[1:]
    if not controllers:
        print("Usage: spawn_controllers.py <ctrl1> [ctrl2] ...", file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("patched_controller_spawner")

    load_cli = node.create_client(LoadController, "/controller_manager/load_controller")
    cfg_cli = node.create_client(ConfigureController, "/controller_manager/configure_controller")
    sw_cli = node.create_client(SwitchController, "/controller_manager/switch_controller")

    print("Waiting for controller_manager services (up to 120s)...")
    for cli, name in [(load_cli, "load"), (cfg_cli, "configure"), (sw_cli, "switch")]:
        if not cli.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            print(f"ERROR: /controller_manager/{name}_controller service not available",
                  file=sys.stderr)
            rclpy.shutdown()
            return 1
    print("All controller_manager services found.")

    for ctrl in controllers:
        print(f"\n--- Spawning {ctrl} ---")

        # Load
        req = LoadController.Request()
        req.name = ctrl
        print(f"  Loading {ctrl}...")
        res = call_with_timeout(node, load_cli, req)
        if res is None:
            print(f"  Load timed out — CM may still be loading; waiting 15s...")
            time.sleep(15)
        elif not res.ok:
            # 'already loaded' comes back as ok=False; treat as success and continue
            print(f"  Load returned not-ok (possibly already loaded) — continuing")
        else:
            print(f"  Loaded.")

        # Configure
        req = ConfigureController.Request()
        req.name = ctrl
        print(f"  Configuring {ctrl}...")
        res = call_with_timeout(node, cfg_cli, req)
        if res is None:
            print(f"  Configure timed out — continuing")
        elif not res.ok:
            print(f"  Configure returned not-ok (possibly already configured) — continuing")
        else:
            print(f"  Configured.")

        # Activate
        req = SwitchController.Request()
        req.activate_controllers = [ctrl]
        req.strictness = SwitchController.Request.BEST_EFFORT
        print(f"  Activating {ctrl}...")
        res = call_with_timeout(node, sw_cli, req)
        if res is None:
            print(f"  Activate timed out")
        elif not res.ok:
            print(f"  Activate returned not-ok")
        else:
            print(f"  {ctrl} is active!")

    print("\nDone spawning controllers.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
