import socketserver
from random import randint

class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        requestForUpdate = self.request.recv(1024)
        
        while requestForUpdate != '':
            self.data = requestForUpdate.strip()
            print("{} wrote:".format(self.client_address[0]))
            print(self.data)
            # just send back the same data, but upper-cased

            self.request.sendall(bytes( str(randint(1,3)) + ",1.0,1.0\n", "utf-8"))
            requestForUpdate=self.request.recv(1024)

if __name__ == "__main__":
    HOST, PORT = "127.0.0.1", 9527

    # Create the server, binding to localhost on port 9999

    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler, False)

    server.allow_reuse_address = True
    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.server_bind()
    server.server_activate()
    server.serve_forever()
