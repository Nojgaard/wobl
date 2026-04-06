from woblpy.common.node import Node

node = Node()
pub = node.add_pub("chatter")
node.add_timer(lambda: node.send_string(pub, "Hello World!"), 1.0)
node.spin()