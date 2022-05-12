import logging
import sys

from helpers.fake_camera import FakeCamera
from sensopart_connector.sensopart import SensoPart


class TestSensopart:

    ip = '127.0.0.1'
    port = 2006
    sensopart: SensoPart = None
    fake_camera: FakeCamera = None

    def setup(self):
        print("Setup")
        self.fake_camera = FakeCamera()

        # Setup logging
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(logging.DEBUG)
        log = logging.getLogger("Sensopart")
        log.handlers = []
        log.addHandler(handler)
        self.sensopart = SensoPart(self.ip, log)

    def teardown(self):
        print("Teardown")
        assert not self.fake_camera.error, self.fake_camera.error

    def connect(self):
        # Start fake camera
        self.fake_camera.run(self.ip, self.port)

        # Connect
        success, msg = self.sensopart.connect()
        assert success, msg

    def test_connect(self):
        success, msg = self.sensopart.connect()
        assert not success, msg

        self.connect()

        success, msg = self.sensopart.connect()
        assert not success, msg
        assert "already connected" in msg

    def test_disconnect(self):
        success, msg = self.sensopart.disconnect()
        assert success
        assert "already closed" in msg

        self.connect()

        assert self.sensopart.connected

        success, msg = self.sensopart.disconnect()
        assert success
        assert "Connection closed" in msg

    def test_trigger(self):
        self.connect()
        self.fake_camera.set_response(['TRG'])
        self.fake_camera.set_replies(['TRGP'])

        success, passed = self.sensopart.trigger()
        assert success
        assert passed == 'P'

    def test_trigger_failed(self):
        self.connect()
        self.fake_camera.set_response(['TRG'])
        self.fake_camera.set_replies(['TRGF'])

        success, passed = self.sensopart.trigger()
        assert success
        assert passed == 'F'

    def test_trigger_without_connection(self):
        assert not self.sensopart.connected

        success, msg = self.sensopart.trigger()
        assert not success
        assert msg == "Camera is not connected."

    def test_extended_trigger(self):
        self.connect()
        self.fake_camera.set_response(['TRX06MyPart'])
        self.fake_camera.set_replies(['TRXP06MyPartR00000000'])

        success, msg, passed, mode = self.sensopart.extended_trigger('MyPart')
        assert success
        assert msg == ''
        assert passed
        assert mode == 'R'

    def test_extended_trigger_without_connection(self):
        assert not self.sensopart.connected

        success, msg, passed, mode = self.sensopart.extended_trigger('MyPart')
        assert not success
        assert msg == "Camera is not connected."
        assert not passed
        assert mode == ''

    def test_set_job(self):
        self.connect()
        self.fake_camera.set_response(['CJB005'])
        self.fake_camera.set_replies(['CJBPT005'])

        success, msg = self.sensopart.change_job(5)
        assert success
        assert "success" in msg

    def test_set_job_failed(self):
        self.connect()
        self.fake_camera.set_response(['CJB000'])
        self.fake_camera.set_replies(['CJBFT005'])

        success, msg = self.sensopart.change_job(0)
        assert not success
        assert "failed" in msg

    def test_set_job_permanent(self):
        self.connect()
        self.fake_camera.set_response(['CJP005'])
        self.fake_camera.set_replies(['CJPPT005'])

        success, msg = self.sensopart.change_job_permanent(5)
        assert success
        assert "success" in msg

    def test_set_job_permanent_failed(self):
        self.connect()
        self.fake_camera.set_response(['CJP000'])
        self.fake_camera.set_replies(['CJPFT005'])

        success, msg = self.sensopart.change_job_permanent(0)
        assert not success
        assert "failed" in msg

    def test_get_job_list(self):
        self.connect()
        self.fake_camera.set_response(['GJL'])
        self.fake_camera.set_replies([
            'GJLP001007007019Find Black Part old037Contour and Calibration Plate (200mm)009SensoPa'
            'rt2018042520180906015Find White Part037Contour and Calibration Plate (200mm)009SensoP'
            'art2018082920180926017Calibration White030Job to calibrate and only that009SensoPart2'
            '018082920180905015Find Black Part003Job009SensoPart2018090620180910024QuickAndDirtyWh'
            'iteDetect011Demo Baumer009SensoPart2018092620180926012DanfossGraco003Job009SensoPart2'
            '020052720200527004Job7003Job009SensoPart2020052720200602'])

        success, msg, jobs = self.sensopart.get_job_list()
        assert success
        assert msg == ''
        assert len(jobs) == 7
        assert jobs[0].name == "Find Black Part old"
