import SoapySDR
from SoapySDR import *
import numpy as np
import time
import statistics
import os
from datetime import datetime
import RPi.GPIO as GPIO

pin = 17
pin_led = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.setup(pin_led, GPIO.OUT, initial=GPIO.LOW)

SAMPLE_RATE = 10000000
GAIN = 100
SAMPLES_PER_READ = 8192
SERIAL_NUMBER = None

BASE_THRESHOLD = 2
MAX_THRESHOLD = 10
ADAPTIVE = True
THRESHOLD_DELTA = 3

STEP = 10_000_000
LISTEN_DURATION = 2

FREQUENCY_RANGES = [
    (902_000_000, 921_000_000, 3),
    (923_000_000, 928_000_000, 3),
    (2_400_000_000, 2_480_000_000, 3),
    (1_070_000_000, 1_370_000_000, 3),
    (5_725_000_000, 5_875_000_000, 3),
]

ignore_counter = {}
ignore_limit = 3
ignore_duration = 300

os.makedirs("logs", exist_ok=True)
log_filename = datetime.now().strftime("logs/log_%Y-%m-%d_%H-%M-%S.txt")


def setup_sdr():
    while True:
        try:
            print("🔌 Сканирование доступных устройств HackRF...")
            devices = SoapySDR.Device.enumerate({'driver': 'hackrf'})
            if len(devices) == 0:
                raise Exception("Устройства HackRF не найдены.")
            sdr = SoapySDR.Device(devices[0])
            sdr.setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE)
            sdr.setGain(SOAPY_SDR_RX, 0, GAIN)
            hw = sdr.getHardwareInfo()
            serial = hw['serial'] if 'serial' in hw else 'unknown'
            print(f"Серийный номер HackRF: {serial}")
            return sdr
        except Exception as e:
            print(f"⚠️ Ошибка подключения к HackRF: {e}. Повторная попытка через 5 секунд...")
            time.sleep(5)


def measure_signal_power(sdr, center_freq):
    try:
        sdr.setFrequency(SOAPY_SDR_RX, 0, center_freq)
        rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        sdr.activateStream(rxStream)
        buff = np.empty(SAMPLES_PER_READ, np.complex64)
        sr = sdr.readStream(rxStream, [buff], len(buff), timeoutUs=1000000)
        sdr.deactivateStream(rxStream)
        sdr.closeStream(rxStream)
        if sr.ret > 0:
            data = buff[:sr.ret]
            fft = np.fft.fftshift(np.fft.fft(data))
            psd = 10 * np.log10(np.abs(fft) ** 2 + 1e-12)
            return np.max(psd)
        else:
            return None
    except Exception:
        return None


def signal_it(fast=False):
    pwm = GPIO.PWM(pin, 2000)
    try:
        repeats = 3 if fast else 3
        for _ in range(repeats):
            GPIO.output(pin_led, GPIO.HIGH)
            pwm.start(50)
            time.sleep(0.1 if fast else 0.25)
            pwm.ChangeDutyCycle(0)
            GPIO.output(pin_led, GPIO.LOW)
            time.sleep(0.05 if fast else 0.15)
    finally:
        pwm.stop()
        GPIO.output(pin_led, GPIO.LOW)


def adai(pin=17):
    pwm = GPIO.PWM(pin, 440)
    melody = [
        (659, 0.18), (880, 0.12), (987, 0.18), (880, 0.12),
        (784, 0.18), (659, 0.18), (880, 0.25), (0, 0.12),
        (987, 0.18), (1046, 0.18), (987, 0.12), (880, 0.25),
        (784, 0.15), (880, 0.12), (659, 0.25)
    ]
    pwm.start(50)
    for freq, dur in melody:
        if freq == 0:
            pwm.ChangeDutyCycle(0)
        else:
            pwm.ChangeDutyCycle(50)
            pwm.ChangeFrequency(freq)
        time.sleep(dur)
    pwm.stop()


def live_listen(sdr, frequency, median_noise, duration=2, samples=10):
    signal_above_threshold = 0
    for _ in range(samples):
        power = measure_signal_power(sdr, frequency)
        if power is not None and power - median_noise > THRESHOLD_DELTA:
            signal_above_threshold += 1
        time.sleep(duration / samples)
    return signal_above_threshold


def should_ignore(freq):
    global ignore_counter
    now = time.time()
    for f in list(ignore_counter.keys()):
        if now - ignore_counter[f]["time"] > ignore_duration:
            del ignore_counter[f]
    if freq in ignore_counter and ignore_counter[freq]["count"] >= ignore_limit:
        print(f"⚠️ Частота {freq//1_000_000} MHz временно игнорируется.")
        return True
    return False


def mark_ignore(freq):
    global ignore_counter
    if freq not in ignore_counter:
        ignore_counter[freq] = {"count": 1, "time": time.time()}
    else:
        ignore_counter[freq]["count"] += 1
        ignore_counter[freq]["time"] = time.time()


def scan_frequency_range(sdr, start_freq, end_freq, threshold):
    readings = {}
    print(f"\nСканирование диапазона {start_freq//1_000_000}-{end_freq//1_000_000} MHz...")
    freq = start_freq
    while freq <= end_freq:
        if should_ignore(freq):
            freq += STEP
            continue
        strength = measure_signal_power(sdr, freq)
        if strength is not None:
            readings[freq] = strength
            if len(readings) >= 3:
                median_noise = statistics.median(list(readings.values()))
                noise_spread = max(readings.values()) - min(readings.values())
                dynamic_threshold = BASE_THRESHOLD + min(noise_spread / 2, MAX_THRESHOLD - BASE_THRESHOLD)
                if 5_700_000_000 <= freq <= 5_900_000_000:
                    dynamic_threshold /= 2
                if strength - median_noise > dynamic_threshold:
                    signal_above_threshold = live_listen(sdr, freq, median_noise, LISTEN_DURATION)
                    if signal_above_threshold >= threshold:
                        if 2_300_000_000 <= freq <= 2_500_000_000 or 5_700_000_000 <= freq <= 5_900_000_000:
                            signal_it()
                        else:
                            signal_it(fast=True)
                            mark_ignore(freq)
        freq += STEP
        time.sleep(0.2)
    return readings


def main():
    try:
        sdr = setup_sdr()
        adai()
        while True:
            for start, end, threshold in FREQUENCY_RANGES:
                scan_frequency_range(sdr, start, end, threshold)
            print("\n\n")
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\n🛑 Завершено пользователем.")


if __name__ == '__main__':
    main()