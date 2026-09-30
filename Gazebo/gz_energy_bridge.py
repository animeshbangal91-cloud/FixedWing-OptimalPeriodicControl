"""Read-only Gazebo transport adapter; run with Ubuntu's system Python."""
import json
import signal
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V

node = Node()
def receive(msg):
    if len(msg.data) == 12:
        print(json.dumps(list(msg.data)), flush=True)

if not node.subscribe(Double_V, '/model/stallion_0/energy_diagnostics', receive):
    raise SystemExit('Could not subscribe to Stallion energy diagnostics')
signal.pause()
