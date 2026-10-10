# Bee@C3LAB cflib baseline

Source: https://github.com/leeebo/crazyflie-lib-python

Baseline commit: `a430283a3b608db7c0b3227aa37d7120858c25ef`

Bee changes configure UDP discovery through `DRONE_URI` and `DRONE_URIS`,
allocate a separate source port per connection, send binary handshake bytes,
respect receive timeouts, and ignore undersized UDP packets.

The UDP transport skips memory enumeration because ESP-Drone's memory service
is compiled out. Crazyflie.close_link accepts `send_stop=False` for read-only
diagnostics; normal GUI disconnects retain the default zero-thrust command.
Disconnect also cancels outstanding retry timers.

A 250 ms CRTP link-sink keepalive prevents ESP-Drone's one-second idle-link
timeout from resetting its logging blocks. This carries no setpoint and does
not refresh the firmware's separate commander watchdog.
