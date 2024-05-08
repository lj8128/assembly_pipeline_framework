from workstation_node import WsNode
from shared_constants import DUMMY_HEAD_WS_NAME

class WsDirectory:
    def __init__(self):
        self._d_head = WsNode(ws_name=DUMMY_HEAD_WS_NAME)
        self._d_tail = WsNode(ws_name='dummy_tail_ws')
        self._d_head.next = self._d_tail
        self._d_tail.prev = self._d_head
        self._ws_dict = {}
        self._ws_dict[DUMMY_HEAD_WS_NAME] = self._d_head
        self._ws_count = 0

    def insert(self, ws_node):
        cur_ws_name = ws_node.ws_name

        if cur_ws_name in self._ws_dict:
            raise ValueError(f'A workstation node with the name '
                    f'{ws_node} already exists in the ws directory! '
                    f'Note: all ws names must be unique.')

        self._back_insert_node(ws_node) 

        self._ws_dict[cur_ws_name] = ws_node

        self._ws_count += 1

    def remove(self, ws_node):
        cur_ws_name = ws_node.ws_name

        if cur_ws_name not in self._ws_dict:
            raise ValueError(f'No workstation with the name {ws_node} '
                    f'exists in the ws directory! ')

        ws_node.prev.next = ws_node.next
        ws_node.next.prev = ws_node.prev

        del self._ws_dict[cur_ws_name]

        self._ws_count -= 1
        
    def get(self, ws_name):
        if ws_name not in self._ws_dict:
            raise ValueError(f'No workstation with the name {ws_node} '
                    f'exists in the ws directory! ')

        return self._ws_dict[ws_name]
    
    def get_ws_dict(self):
        return self._ws_dict 

    def size(self):
        return self._ws_count

    def _back_insert_node(self, ws_node):
        tmp = self._d_tail.prev
        self._d_tail.prev = ws_node
        ws_node.next = self._d_tail
        tmp.next = ws_node
        ws_node.prev = tmp

