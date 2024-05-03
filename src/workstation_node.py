class WsNode:
    def __init__(self, next=None, prev=None, ws_name, latency=0.0):
        self._next = next
        self._prev = prev
    
        self.ws_name = ws_name
        self.latency = latency
 
