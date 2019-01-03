from __future__ import print_function, unicode_literals, division

from ctypes import (Structure, c_ulong, c_long, c_int, c_uint, c_float,
                    byref, oledll, create_string_buffer)


_error_codes = {
    20001: "DRV_ERROR_CODES",
    20002: "DRV_SUCCESS",
    20003: "DRV_VXNOTINSTALLED",
    20006: "DRV_ERROR_FILELOAD",
    20007: "DRV_ERROR_VXD_INIT",
    20010: "DRV_ERROR_PAGELOCK",
    20011: "DRV_ERROR_PAGE_UNLOCK",
    20013: "DRV_ERROR_ACK",
    20024: "DRV_NO_NEW_DATA",
    20026: "DRV_SPOOLERROR",
    20034: "DRV_TEMP_OFF",
    20035: "DRV_TEMP_NOT_STABILIZED",
    20036: "DRV_TEMP_STABILIZED",
    20037: "DRV_TEMP_NOT_REACHED",
    20038: "DRV_TEMP_OUT_RANGE",
    20039: "DRV_TEMP_NOT_SUPPORTED",
    20040: "DRV_TEMP_DRIFT",
    20050: "DRV_COF_NOTLOADED",
    20053: "DRV_FLEXERROR",
    20066: "DRV_P1INVALID",
    20067: "DRV_P2INVALID",
    20068: "DRV_P3INVALID",
    20069: "DRV_P4INVALID",
    20070: "DRV_INIERROR",
    20071: "DRV_COERROR",
    20072: "DRV_ACQUIRING",
    20073: "DRV_IDLE",
    20074: "DRV_TEMPCYCLE",
    20075: "DRV_NOT_INITIALIZED",
    20076: "DRV_P5INVALID",
    20077: "DRV_P6INVALID",
    20083: "P7_INVALID",
    20089: "DRV_USBERROR",
    20091: "DRV_NOT_SUPPORTED",
    20099: "DRV_BINNING_ERROR",
    20990: "DRV_NOCAMERA",
    20991: "DRV_NOT_SUPPORTED",
    20992: "DRV_NOT_AVAILABLE"
}


class AndorError(Exception):
    def __str__(self):
        return "%i: %s" % (self.args[0], _error_codes[self.args[0]])

    @classmethod
    def check(cls, status):
        if status != 20002:
            raise cls(status)


class AndorCapabilities(Structure):
    _fields_ = [
        ("size", c_ulong),
        ("acq_modes", c_ulong),
        ("read_modes", c_ulong),
        ("trigger_modes", c_ulong),
        ("camera_type", c_ulong),
        ("pixel_mode", c_ulong),
        ("set_functions", c_ulong),
        ("get_functions", c_ulong),
        ("features", c_ulong),
        ("pci_card", c_ulong),
        ("em_gain_capability", c_ulong),
        ("ft_read_modes", c_ulong),
    ]


_dll = None


def load_dll(path):
    global _dll
    assert not _dll
    _dll = oledll.LoadLibrary(path)


def get_available_cameras():
    n = c_long()
    AndorError.check(_dll.GetAvailableCameras(byref(n)))
    return n.value


def set_current_camera(c):
    AndorError.check(_dll.SetCurrentCamera(c))


_current_handle = None


def make_current(c):
    global _current_handle
    if c != _current_handle:
        set_current_camera(c)
        _current_handle = c


class Camera:
    @classmethod
    def by_index(cls, i):
        h = c_long()
        AndorError.check(_dll.GetCameraHandle(i, byref(h)))
        return cls(h.value)

    @classmethod
    def first(cls):
        return cls.by_index(0)

    def _make_current(self):
        make_current(self.handle)

    def __init__(self, handle):
        self.handle = handle

    @staticmethod
    def _best_index(values, search):
        dev = None
        for index, value in enumerate(values):
            if dev is None or abs(value - search) < dev:
                best = index
                dev = abs(value - search)
        return best

    def initialize(self, ini):
        self._make_current()
        AndorError.check(_dll.Initialize(ini))

    def shutdown(self):
        self._make_current()
        AndorError.check(_dll.ShutDown())

    def get_capabilities(self):
        self._make_current()
        caps = AndorCapabilities()
        AndorError.check(_dll.GetCapabilities(byref(caps)))
        return caps

    def get_detector(self):
        self._make_current()
        x, y = c_long(), c_long()
        AndorError.check(_dll.GetDetector(byref(x), byref(y)))
        return x.value, y.value

    def get_head_model(self):
        self._make_current()
        m = create_string_buffer(32)
        AndorError.check(_dll.GetHeadModel(m))
        return m.value

    def get_hardware_version(self):
        self._make_current()
        card = c_uint()
        flex10k = c_uint()
        dummy1 = c_uint()
        dummy2 = c_uint()
        firmware = c_uint()
        build = c_uint()
        AndorError.check(_dll.GetHardwareVersion(
            byref(card), byref(flex10k), byref(dummy1), byref(dummy2),
            byref(firmware), byref(build)))
        return dict(card=card.value, flex10k=flex10k.value,
                    firmware=firmware.value, build=build.value)

    def set_vs_amplitude(self, amplitude):
        self._make_current()
        AndorError.check(_dll.SetVSAmplitude(amplitude))

    def get_vs_speeds(self):
        self._make_current()
        n = c_int()
        AndorError.check(_dll.GetNumberVSSpeeds(byref(n)))
        speed = c_float()
        for i in range(n.value):
            AndorError.check(_dll.GetVSSpeed(i, byref(speed)))
            yield speed.value

    def set_vs_speed(self, speed):
        if isinstance(speed, float):
            speeds = self.get_vs_speeds()
            speed = self._best_index(speeds, speed)
        self._make_current()
        AndorError.check(_dll.SetVSSpeed(speed))

    def get_number_of_adc_channels(self):
        self._make_current()
        n = c_int()
        AndorError.check(_dll.GetNumberADChannels(byref(n)))
        return n.value

    def get_adc_channel(self):
        self._make_current()
        c = c_int()
        AndorError.check(_dll.GetADCChannel(byref(c)))
        return c.value

    def set_adc_channel(self, channel):
        self._make_current()
        AndorError.check(_dll.SetADChannel(channel))
        AndorError.check(_dll.SetOutputAmplifier(channel))

    def get_bit_depth(self, channel=None):
        self._make_current()
        if channel is None:
            return [self.get_bit_depth(c)
                    for c in range(self.get_number_of_adc_channels())]
        else:
            b = c_int()
            AndorError.check(_dll.GetBitDepth(channel, byref(b)))
            return b.value

    def set_fan_mode(self, mode):
        """0 = full, 1 = low, 2 = off"""
        self._make_current()
        AndorError.check(_dll.SetFanMode(mode))

    def cooler(self, on=True):
        self._make_current()
        if on:
            AndorError.check(_dll.CoolerON())
        else:
            AndorError.check(_dll.CoolerOFF())

    def get_temperature_range(self):
        self._make_current()
        min, max = c_int(), c_int()
        AndorError.check(_dll.GetTemperatureRange(byref(min), byref(max)))
        return min.value, max.value

    def get_temperature(self):
        self._make_current()
        t = c_int()
        status = _dll.GetTemperature(byref(t))
        return t.value, _error_codes[status]

    def set_temperature(self, t):
        self._make_current()
        AndorError.check(_dll.SetTemperature(int(t)))

    def get_preamp_gains(self):
        self._make_current()
        n = c_int()
        AndorError.check(_dll.GetNumberPreAmpGains(byref(n)))
        gain = c_float()
        for i in range(n.value):
            AndorError.check(_dll.GetPreAmpGain(i, byref(gain)))
            yield gain.value

    def set_preamp_gain(self, gain):
        if isinstance(gain, float):
            gains = self.get_preamp_gains()
            gain = self._best_index(gains, gain)
        self._make_current()
        AndorError.check(_dll.SetPreAmpGain(gain))

    def get_em_gain_range(self):
        self._make_current()
        low, high = c_int(), c_int()
        AndorError.check(_dll.GetEMGainRange(byref(low), byref(high)))
        return low.value, high.value

    def set_em_gain_mode(self, mode):
        self._make_current()
        AndorError.check(_dll.SetEMGainMode(mode))

    def set_emccd_gain(self, gain):
        self._make_current()
        AndorError.check(_dll.SetEMCCDGain(gain))

    def get_status(self):
        self._make_current()
        state = c_int()
        AndorError.check(_dll.GetStatus(byref(state)))
        return state.value

    def start_acquisition(self):
        self._make_current()
        AndorError.check(_dll.StartAcquisition())

    def abort_acquisition(self):
        self._make_current()
        AndorError.check(_dll.AbortAcquisition())

    def set_shutter(self, a, b, c, d):
        self._make_current()
        AndorError.check(_dll.SetShutter(a, b, c, d))

    def shutter(self, open=True):
        self.set_shutter(0, 1 if open else 2, 0, 0)

    def get_acquisition_timings(self):
        self._make_current()
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        AndorError.check(_dll.GetAcquisitionTimings(
            byref(exposure), byref(accumulate), byref(kinetic)))
        return exposure.value, accumulate.value, kinetic.value

    def set_acquisition_mode(self, frames=0):
        self._make_current()
        if frames == 1:
            AndorError.check(_dll.SetAcquisitionMode(1))
        elif frames > 1:
            AndorError.check(_dll.SetAcquisitionMode(3))
            AndorError.check(_dll.SetNumberAccumulations(1))
            AndorError.check(_dll.SetAccumulationCycleTime(0))
            AndorError.check(_dll.SetNumberKinetics(frames))
        elif frames == 0:
            AndorError.check(_dll.SetAcquisitionMode(5))

    def set_baseline_clamp(self, active=True):
        self._make_current()
        AndorError.check(_dll.SetBaselineClamp(int(active)))

    def set_exposure_time(self, time):
        self._make_current()
        AndorError.check(_dll.SetExposureTime(c_float(time)))

    def set_frame_transfer_mode(self, mode):
        self._make_current()
        AndorError.check(_dll.SetFrameTransferMode(mode))

    def get_hs_speeds(self, channel=None):
        self._make_current()
        if channel is None:
            for channel in range(self.get_number_of_adc_channels()):
                for _ in self.get_hs_speeds(channel):
                    yield channel, _
        else:
            n = c_int()
            AndorError.check(_dll.GetNumberHSSpeeds(channel, 0, byref(n)))
            speed = c_float()
            for i in range(n.value):
                AndorError.check(_dll.GetHSSpeed(channel, 0, i, byref(speed)))
                yield speed.value

    def set_hs_speed(self, speed):
        if isinstance(speed, float):
            speeds = self.get_hs_speeds(self.get_adc_channel())
            speed = self._best_index(speeds, speed)
        self._make_current()
        AndorError.check(_dll.SetHSSpeed(0, speed))

    def set_kinetic_cycle_time(self, time):
        self._make_current()
        AndorError.check(_dll.SetKineticCycleTime(c_float(time)))

    def set_read_mode(self, mode):
        self._make_current()
        AndorError.check(_dll.SetReadMode(mode))

    def set_image(self, bin, roi):
        """
        bin: (bin_x, bin_y)
        roi: (x1, x2, y1, y2)
        """
        self._make_current()
        AndorError.check(_dll.SetImage(bin[0], bin[1],
                                       roi[0], roi[1], roi[2], roi[3]))
        nx = (roi[1] - roi[0] + 1)//bin[0]
        ny = (roi[3] - roi[2] + 1)//bin[1]
        self.shape = nx, ny
        self.size = nx * ny

    def set_trigger_mode(self, mode):
        self._make_current()
        AndorError.check(_dll.SetTriggerMode(mode))

    def set_fast_ext_trigger(self, mode):
        self._make_current()
        AndorError.check(_dll.SetFastExtTrigger(mode))

    def get_number_new_images(self):
        self._make_current()
        first, last = c_long(), c_long()
        AndorError.check(_dll.GetNumberNewImages(byref(first), byref(last)))
        return first.value, last.value

    def get_oldest_image16(self):
        self._make_current()
        buffer = create_string_buffer(2*self.size)
        AndorError.check(_dll.GetOldestImage16(buffer, self.size))
        return buffer

    def get_images16(self, first_last=None):
        self._make_current()
        if first_last is None:
            try:
                first, last = self.get_number_new_images()
            except AndorError:
                return
        else:
            first, last = first_last
        n = last - first + 1
        if n == 0:
            return
        buf = create_string_buffer(2*n*self.size)
        valid_first, valid_last = c_long(), c_long()
        AndorError.check(_dll.GetImages16(
            first, last, buf, n*self.size,
            byref(valid_first), byref(valid_last)))
        for i in range(n):
            yield buf[2*self.size*i:2*self.size*(i + 1)]
