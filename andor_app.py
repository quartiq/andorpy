from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import time
import logging
import os

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

import andor


logger = logging.getLogger(__name__)


class AndorPlot:
    def __init__(self, path):
        self.init_win()
        self.init_camera(path)

    def init_win(self):
        self.win = QtGui.QMainWindow()
        self.win.show()
        self.win.resize(600, 400)
        self.win.setWindowTitle("Andor")
        self.img = pg.ImageView()
        self.win.setCentralWidget(self.img)

    def init_camera(self, path):
        andor.load_dll(os.path.join(path, "atmcd32d.dll"))
        self.camera = c = andor.Camera.first()
        c.initialize(os.path.join(path, "Detector.ini"))
        logger.info(c.get_capabilities())
        logger.info(c.get_hardware_version())
        logger.info(c.get_head_model())
        logger.info(c.get_detector())
        logger.info(c.get_temperature())
        logger.info(c.get_temperature_range())
        logger.info(c.get_acquisition_timings())
        logger.info(list(c.get_hs_speeds()))
        logger.info(list(c.get_vs_speeds()))
        logger.info(list(c.get_preamp_gains()))
        logger.info(c.get_em_gain_range())

        c.cooler(True)
        c.set_fan_mode(0)  # full
        c.set_temperature(-20)  # C
        c.set_read_mode(4)  # image
        c.set_trigger_mode(0)  #
        c.set_preamp_gain(0)  # 1.
        c.set_baseline_clamp(True)
        c.set_vs_speed(2)  # 1.9us
        c.set_vs_amplitude(0)
        c.set_hs_speed(0)  # 35MHz
        c.set_image((4, 4), (1, 1000, 1, 1000))
        c.set_frame_transfer_mode(0)
        c.set_acquisition_mode(frames=0)
        c.set_exposure_time(.3)
        c.set_em_gain_mode(0)
        c.set_emccd_gain(64)
        c.shutter(True)

    def start_camera(self):
        self.camera.start_acquisition()

    def process_images(self):
        QtCore.QTimer.singleShot(100, self.process_images)
        try:
            frame = self.camera.get_oldest_image16()
            logger.debug("got image")
        except andor.AndorError:
            logger.debug("no image")
            return
        # x,y coord system view
        frame = np.frombuffer(frame, np.uint16).reshape(
            self.camera.shape)[:, ::-1]
        # T[::-1, ::-1] # -y,x operator view
        self.img.setImage(frame, autoRange=False, autoLevels=False,
                          autoHistogramRange=False)

    def stop_camera(self):
        self.camera.abort_acquisition()

    def deinit_camera(self):
        self.camera.shutter(False)
        self.camera.cooler(False)
        self.camera.set_fan_mode(2)
        self.camera.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app = QtGui.QApplication([])
    cam = AndorPlot("c:/Program Files/Andor Driver Pack 2/")
    try:
        cam.start_camera()
        time.sleep(.5)
        cam.process_images()
        cam.img.autoRange()
        cam.img.autoLevels()
        cam.img.setLevels(450, 650)
        QtGui.QApplication.instance().exec_()
    finally:
        cam.stop_camera()
        cam.deinit_camera()
