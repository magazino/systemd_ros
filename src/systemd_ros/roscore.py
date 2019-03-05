import socket

from rosgraph.xmlrpc import SilenceableXMLRPCRequestHandler
from rosgraph.xmlrpc import ThreadingXMLRPCServer
from rosmaster.main import rosmaster_main
from six.moves.xmlrpc_server import SimpleXMLRPCServer
from systemd.daemon import listen_fds


def patched_init(self, addr, log_requests=1):
    self.request_queue_size = min(socket.SOMAXCONN, 256)
    SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler,
                                log_requests, bind_and_activate=False)
    self.socket = socket.fromfd(listen_fds()[0],
                                self.address_family, self.socket_type)


def main():
    ThreadingXMLRPCServer.__init__ = patched_init
    rosmaster_main()
