class WsNode:
    def __init__(self, ws_name, next=None, prev=None, latency=0.0):
        self.next = next
        self.prev = prev
    
        self.ws_name = ws_name
        self.latency = latency
 
