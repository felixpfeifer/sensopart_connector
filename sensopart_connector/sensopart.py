import logging
import socket
import time
import sys
import cv2
import numpy as np


class Job:

    def __init__(self, name: str, author: str, description: str, idx: int, date: str):
        self.name = name
        self.author = author
        self.description = description
        self.idx = idx
        self.date = date

    def __str__(self) -> str:
        return f'<name={self.name}, author={self.author}, ' \
               f'description={self.description}, idx={self.idx}, date={self.date}>'


class SensoPart:

    CONNECT_TIMEOUT = 5
    REQUEST_PORT = 2006

    def __init__(self, ip: str, log: logging.Logger):
        self.ip = ip
        self.log = log

        self.connected = False
        self.request_socket: socket.socket = None

        self.log.info("Initialized")

    def __del__(self):
        self.log.info("Closing down...")
        self.disconnect()

    def connect(self) -> (bool, str):
        if self.connected:
            return False, "Camera is already connected."

        try:
            self.log.info(f"Connecting to request interface at: {self.ip}:{self.REQUEST_PORT}...")
            self.request_socket = socket.create_connection((self.ip, self.REQUEST_PORT),
                                                           timeout=self.CONNECT_TIMEOUT)
            msg = f"Connected to SensoPart camera at {self.ip}"
            self.log.info(msg)
            self.connected = True
            return True, msg
        except socket.error as e:
            msg = f"Could not connect to camera at {self.ip}. Error: {e}"
            self.log.error(msg)
            self.connected = False
            return False, msg

    def disconnect(self) -> (bool, str):
        msg = "Connection was already closed!"
        self.connected = False
        if self.request_socket:
            self.request_socket.close()
            self.request_socket = None
            msg = "Connection closed to SensoPart camera."
        self.log.warning(msg)
        return True, msg

    def change_job(self, job_number: int) -> (bool, str):
        return self._set_job(f'CJB{job_number:03d}')

    def change_job_permanent(self, job_number: int) -> (bool, str):
        return self._set_job(f'CJP{job_number:03d}')

    def _set_job(self, request: str) -> (bool, str):
        success, message = self.send_message(request)
        if not success:
            return success, message
        response = self.receive_message(8).decode()
        self.log.info(f"Response: {response}")
        success = True if response[3] == 'P' else False
        mode = 'Triggered' if response[4] == 'T' else 'Free run'
        active_job = int(response[5:8])
        return success, f"Changed job {'successful' if success else 'failed'}. " \
                        f"Active job: {active_job}. Mode: {mode}"

    def change_job_by_name(self, job_name: str) -> (bool, str):
        success, message = self.send_message(f'CJN2{len(job_name):03d}{job_name}')
        if not success:
            return success, message
        response = self.receive_message(8).decode()
        self.log.info(f"Response: {response}")
        success = True if response[3] == 'P' else False
        error_code = int(response[4:7])
        mode = 'Triggered' if response[7] == 'T' else 'Free run'

        return success, f"Changed job {'successful' if success else 'failed'}. " \
                        f"Error code: {error_code}. Mode: {mode}"

    def get_job_list(self) -> (bool, str, [Job]):
        success, message = self.send_message('GJL')
        if not success:
            return success, message, []
        response = self.receive_message(13).decode()
        self.log.info(f"Response: {response}")
        passed = response[3]
        version = int(response[4:7])
        n_jobs = int(response[7:10])
        active_job = int(response[10:13])
        self.log.info(f"Passed: {passed}, version: {version}, "
                      f"#jobs: {n_jobs}, active_job: {active_job}")
        jobs = []
        for i in range(n_jobs):
            n_char_name = int(self.receive_message(3).decode())
            name = self.receive_message(n_char_name).decode()
            m = int(self.receive_message(3).decode())
            description = self.receive_message(m).decode()
            k = int(self.receive_message(3).decode())
            author = self.receive_message(k).decode()
            date = self.receive_message(16).decode()
            # modified = self.receive_message(19).decode()

            job = Job(name, author, description, i + 1, date)
            jobs.append(job)
            self.log.info(f"Job: {job}")

        return True, message, jobs

    def trigger(self) -> (bool, str):
        success, message = self.send_message('TRG')
        if not success:
            return success, message
        response = self.receive_message(4).decode()
        self.log.info(f"[trigger] Response: {response}")
        return True, response[3]

    def extended_trigger(self, data: str) -> (bool, str, bool, str):
        if len(data) > 99:
            return False, "len(data) > 99!", False, ''
        success, msg = self.send_message(f'TRX{len(data):02d}{data}')
        if not success:
            return success, msg, False, ''

        # Retrieve first part
        response = self.receive_message(6).decode()
        success = True if response[3] == 'P' else False
        n = int(response[5:])
        self.log.info(f"[extended_trigger] Response: {response}")
        self.log.debug(f"[extended_trigger] Response: {response}, success: {success}, n={n}")

        # Retrieve second part
        response = self.receive_message(n + 9).decode()
        data_return = response[0:n]
        mode = response[n]
        n = int(response[n+1:])
        self.log.debug(f"[extended_trigger] Response: {response}, "
                       f"data={data_return}, mode={mode}, n={n}")

        # Retrieve result data
        result_data = self.receive_message(n).decode()
        self.log.debug(f"[extended_trigger] Result data: {result_data}")
        return True, result_data, success, mode

    def get_image(self) -> (bool, str, np.ndarray):
        success, msg = self.send_message(f'GIM0')
        if not success:
            return success, msg, np.empty(0)

        response = self.receive_message(15, print_received=True)
        self.log.info(f"[get_image] Response: {response.decode()}")
        rows, cols = int(response[7:11]), int(response[11:15])
        self.log.debug(f"[get_image] Rows: {rows}, cols: {cols}")
        data = self.receive_message(rows * cols)
        nparr = np.fromstring(data, np.uint8)
        nparr = nparr.reshape((rows, cols))

        return True, '', nparr

    def send_message(self, message: str) -> (bool, str):
        if not self.connected:
            return False, "Camera is not connected."
        try:
            self.log.info(f"Sending request: {message}")
            self.request_socket.send(message.encode())
            return True, ''
        except Exception as e:
            self.log.error(f"Some error: {e}")
            self.connected = False
            return False, ''

    def receive_message(self, length: int, print_received=False) -> bytes:
        received = 0
        data = b''
        while received < length:
            response = self.request_socket.recv(length)
            data += response
            received += len(response)
            if print_received:
                self.log.debug(f"Received: [{response}]")
        return data


if __name__ == '__main__':
    # Setup logging
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    root = logging.getLogger("SensoPart")
    root.setLevel(logging.DEBUG)
    root.addHandler(handler)

    # Init camera
    # camera = SensoPart('192.168.100.7', root)
    camera = SensoPart('192.168.100.8', root)

    # Call some commands
    root.info("### Calling connect ###")
    camera.connect()
    root.info("### Trigger ###")
    camera.trigger()
    root.info("### Waiting ###")
    time.sleep(0.5)
    root.info("### Get image ###")
    _, _, img = camera.get_image()
    cv2.imshow('frame', img)
    cv2.waitKey()
    root.info("### Get job list ###")
    camera.get_job_list()
    root.info("### Extended Trigger ###")
    camera.extended_trigger('MyPart')
    root.info("### Waiting ###")
    time.sleep(0.5)
    root.info("### Calling disconnect ###")
    camera.disconnect()
