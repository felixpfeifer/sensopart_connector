import socket
import threading
import time
import queue


class FakeCamera:

    def __init__(self):
        self.error = False
        self.client = None
        self.connected = False
        self.replies = queue.Queue()
        self.responses = queue.Queue()

    def set_response(self, responses: [str]):
        for response in responses:
            self.responses.put(response)

    def set_replies(self, replies: [str]):
        for reply in replies:
            self.replies.put(reply)

    def run(self, ip: str, port: int):
        server_thread = threading.Thread(target=self._run, args=(ip, port,), daemon=True)
        server_thread.start()
        time.sleep(0.01)

    def _run(self, ip: str, port: int):
        # Run a server to listen for a connection and then close it
        print(f"[FakeCamera] Creating a fake server on: {ip}:{port}")
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((ip, port))
        server_sock.listen()
        self.client, _ = server_sock.accept()
        self.connected = True

        client_thread = threading.Thread(target=self._handle_client, daemon=True)
        client_thread.start()

    def _handle_client(self):
        while self.connected:
            try:
                response = self.client.recv(1024)
                response = response.decode('utf-8') if response else ''
                if response == '':
                    print("[FakeCamera] Received empty message")
                    self.connected = False
                else:
                    print(f"[FakeCamera] Received: {response}")
                    expected_response = self.responses.get_nowait()
                    if response != expected_response:
                        print(f"[FakeCamera] !!!ERROR!!! Expected response: {expected_response}, "
                              f"response: {response}")
                        self.error = True
                    reply = self.replies.get_nowait().encode('utf-8')
                    self.client.send(reply)
                    print(f"[FakeCamera] Send: {reply}")
            except Exception as e:
                print(f"[FakeCamera] Error: {e}")
                self.connected = False
        self.client.close()

    def get_free_port(self):
        s = socket.socket(socket.AF_INET, type=socket.SOCK_STREAM)
        s.bind(('localhost', 0))
        address, port = s.getsockname()
        s.close()
        return port
