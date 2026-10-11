# Standalone ODrive ROS2 node

This packages serves as a standalone ROS2 node to control the ODrive via CAN.

For information about installation, prerequisites and getting started, check out the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Interface

### Parameters

* `node_id`: The node_id of the device this node will attach to
* `interface`: the network interface name for the can bus
* `axis_idle_on_shutdown`: Whether to send ODrive an IDLE request on graceful node termination (default: `false`). This is a final CAN write, not a confirmed park operation; callers that need confirmation must request IDLE and check fresh status before terminating the node.

### Subscribes to

* `/control_message`: Input setpoints for the ODrive.

  The ODrive will interpret the values of input_pos, input_vel and input_torque depending on the control mode. 

  For example: In velocity control mode (2) input_pos is ignored, and input_torque is used as a feedforward term.

  **Note:** When changing `input_mode` or `control_mode`, it is advised to set the ODrive to IDLE before doing so. Changing these values during CLOSED_LOOP_CONTROL is not advised.

### Publishes

* `/odrive_status`: Provides ODrive/system level status updates.

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `error_msg_rate_ms`
  - `temperature_msg_rate_ms`
  - `bus_voltage_msg_rate_ms`

  The ROS node will wait until one of each of these CAN messages has arrived before it emits a message on the `odrive_status` topic. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

* `/controller_status`: Provides Controller level status updates. 

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `heartbeat_msg_rate_ms`
  - `encoder_msg_rate_ms`
  - `iq_msg_rate_ms`
  - `torques_msg_rate_ms`

  The ROS node will wait until one of each of these CAN messages has arrived before it emits a message on the `controller_status` topic. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

### Services

* `/request_axis_state`: Sets the axes requested state.

  Input/output parameters: [`odrive_can/srv/AxisState`](srv/AxisState.srv)

  When requesting CLOSED_LOOP_CONTROL or IDLE, the service returns after entering
  the new axis state. When requesting other states (e.g. ENCODER_OFFSET_CALIBRATION),
  the service waits for the axis state to complete before it returns.

  This service requires regular heartbeat messages from the ODrive to function properly. See [configuration instructions](https://docs.odriverobotics.com/v/latest/guides/ros-package.html#setting-up-the-odrive).

  A pending request is cancelled when ROS shuts down, even if no more heartbeats arrive. A shutdown-cancelled request does not send a response.

  The heartbeat wait begins only after the required CAN writes succeed. A failed state write (or prerequisite `clear_errors` write for a non-IDLE request) is logged and sends no success response, even if the current heartbeat already reports the requested state. Clients must bound their wait.

  If the requested state is anything other than IDLE, this sends a `clear_errors` request to the ODrive (see below) before sending the state request.

  **Example** – request CLOSED_LOOP_CONTROL from the CLI (replace `[odrive_node_name]` with the name of your ODrive node instance):

  ```bash
  ros2 service call [odrive_node_name]/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
  ```

* `/clear_errors`: Manual service call to clear disarm_reason and procedure_result, reset the LED color and re-arm the brake resistor if applicable. See also [`clear_errors()`](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.clear_errors).

  This does not affect the axis state.

  The service replies only after the CAN frame has been written successfully. A failed write is logged and sends no success response; clients must bound their wait. Successful transmission does not confirm that the controller cleared its errors: check fresh controller status for that.

  If the axis dropped into IDLE because of an error and the intent is to re-enable it, call `/request_axis_state`
  instead with CLOSED_LOOP_CONTROL, which clears errors automatically.

### Data Types

All of the Message/Service fields are directly related to their corresponding CAN message. For more detailed information about each type, and how to interpet the data, please refer to the [ODrive CAN protocol documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#messages).

Enum types and their corresponding integer values are listed in the ODrive API reference:

- [AxisState](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState)
- [ControlMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode)
- [InputMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode)

If you have the `odrive` Python package installed (not mandatory for this ROS node), you translate enums like this:

```py
from odrive.enums import AxisState
print(AxisState.CLOSED_LOOP_CONTROL) # 8
print(AxisState(8).name) # CLOSED_LOOP_CONTROL
```

## Shutdown and lifetime

`ODriveCanNode` owns its CAN worker. On SIGINT, SIGTERM, or normal node destruction it stops the event loop through a dedicated nonblocking eventfd and joins the worker before issuing the optional final IDLE request, removing all three command-event registrations, or closing the CAN socket. The event loop must outlive the node, and the ROS executor must stop before node destruction or an explicit `deinit()`. Repeated cleanup and partial-initialization cleanup are safe.

`EpollEventLoop` registration/removal is confined to the event-loop thread or a stopped loop. Only `request_stop()` is cross-thread; it permanently stops that loop instance. Callbacks and their embedded event contexts stay alive until the worker is joined, including when a socket removes itself after an interface error.

### Isolated shutdown smoke

Use a new virtual CAN interface, never a physical motor bus. Build the `odrive_can` package first, source `install/setup.bash`, and run in an unused ROS domain/namespace:

```bash
sudo modprobe vcan
sudo ip link add vcan_shutdown type vcan
sudo ip link set vcan_shutdown up
export ROS_DOMAIN_ID=177
node="$(ros2 pkg prefix odrive_can)/lib/odrive_can/odrive_can_node"
for signal in INT TERM; do
  "$node" --ros-args -r __ns:=/shutdown_smoke \
    -p interface:=vcan_shutdown -p axis_idle_on_shutdown:=false &
  pid=$!
  sleep 1
  kill -"$signal" "$pid"
  wait "$pid"
done
sudo ip link delete vcan_shutdown
```

Each node should exit normally without `terminate`, an abort, or waiting for CAN traffic. Repeat with `axis_idle_on_shutdown:=true` only on this virtual interface to exercise the final-write path. Also test shutdown during an IDLE service call with no incoming heartbeat, successful service completion using synthetic heartbeat traffic, interface removal during traffic, and an interface-down `clear_errors` request (it must log the failed write and must not reply successfully).

Verified on SCR node1 with a dedicated virtual CAN interface: SIGINT and SIGTERM
exit normally with `axis_idle_on_shutdown` both false and true; an IDLE service
waiting on a silent bus cancels on shutdown; failed IDLE and ClearErrors writes
log failures without successful replies; invalid-interface startup exits 1
without an abort. The full testing launch was also exercised with synthetic
board telemetry in isolated ROS domain 177. No physical motor bus was used.


