# Summer 2026 Update - slvrov_ros Nodes

## Nodes

These are our current nodes:
- joystick_calibrator
- joystick_logic
- pca9685_node
- pca9685_pin_config_server
- pca9685_pin_config_client
- thruster_bridge

They all manage core features, and so should all go into the updates' new slvrov_core_nodes_python package.

### joystick_calibrator

This node was written entirely by AI, and so I don't fully understand how it works. So, over the duration of the update, I want to understand how it works.

In here, there is an option to store the file as a YAML or a JSON. We need to choose a single format, I think that I would prefer JSOn in the case that we need to send these up to a web server (JSONs are easily parsable with js).

### joystick logic

This node was written entirely by AI, and so I don't fully understand how it works. So, over the duration of the update, I want to understand how it works.

### AI nodes

The AI nodes, like joystick_calibrator and joystick_logic are very convoluted and disorganized. We need to go through these to refactor and remove uneeded features.

### pca9685_node

Recently, this node has caused a problem, taking up to 80% CPU. My best guess it that it blocks the CPU when it is writing to the i2c bus so often, so we need to fix it with:
- lower js update hz
- only send pwm when it has changed by a certain threshold
- use non-blocking I/O? (async or threading? I found something for async i2c [here](https://github.com/jabdoa2/smbus2_asyncio/blob/master/smbus2_asyncio/__init__.py))

Another option is that we could find a 3rd-party solution. I found [this](https://github.com/vertueux/i2c_pwm_board) repo, which interfaces with the pca9685 chip, but I don't know if it would solve our problem and our system is already designed around our pca9685-node system.

### pca9685_pin_config_server, pca9685_pin_config_client

These nodes control the pin mapping for the pca9685 chip. This ensures that the surface-side logic (joystics, etc.) don't need to know the hardware layout, but only the designated names of the servos/motors. For that reason, I would actually say we should rename these to "mappings" stead of "configs".

We need to add an edit and a delet service/function here, because the only option for changing them is going into the file (which isn't always easy) or deleting the file and starting again.

### thruster_bridge

This node right now relays the messages from the joystick_logic node to the pca9685. It's not needed, as it just republoishes the same messages. This should be removed in the update.

### Mappings

Many of these nodes store mapping files on disk. I think that these should either be managed by user if they wish, or stored inside of a config directory in the core package (make sure they're untracked in git).
