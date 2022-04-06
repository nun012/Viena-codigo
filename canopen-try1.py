import canopen
from can import CanError

node={'node0':1,'node1':2,'node2':3}
object_dictionary={'command':601}

def create_network(_network=None, debug=False):
    if not _network:
        network = canopen.Network()
    else:
        network = _network
    return network

def create_node(network, nodeID):
    try:
        node = network.add_node(nodeID, object_dictionary=None)
        network.connect(channel='can0', bustype = 'socketcan')
        _connected = True
    except Exception as e:
        print("Exception caught:{0}".format(str(e)))
    finally:
        return node, _connected

def read_object(node, index, subindex):
    try:
        return node.sdo.upload(index, subindex)
    except Exception as e:
        print("Exception caught:{0}".format(str(e)))
        return None


def write_object(node, index, subindex, data):
    try:
        node.sdo.download(index, subindex, data)
        return True
    except canopen.SdoAbortedError as e:
        print("Code 0x{:08X}".format(e.code))
        return False
    except canopen.SdoCommunicationError:
        print('SdoAbortedError: Timeout or unexpected response')
        return False

def get_no_fault(node):
    index=0x5300
    subindex=0x1
    no_faults=read_object(node,index,subindex)
    no_faults=int.from_bytes(no_faults,'little')
    return no_faults

def select_fault(code, fault_no):
    index=0x5300
    subindex=0x2
    data=fault_no.to_bytes(2,'little')
    return write_object(node, index, subindex, data)

def get_fault_id(node):
    index=0x5300
    subindex=0x3
    fault_id=read_object(node,index, subindex)
    fault_id=int.from_bytes(fault_id,'little')
    return fault_id

def get_fault(node):
    no_faults=get_no_fault(node)
    for fault in range (no_faults):
        select_fault(node, fault)
        fault_id=get_fault_id(node)
        print("Your fault ID is 0x{:04x}".format(fault_id))

def apples():
    network = create_network()
    node, connected = create_node(network,1)
    if connected:
        get_fault(node)
    return

if __name__== '__main__':
    apples()
    
