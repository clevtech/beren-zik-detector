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
    (2_400_000_000, 2_480_000_000, 3),
    (902_000_000, 921_000_000, 3),
    (923_000_000, 928_000_000, 3),
    (2_400_000_000, 2_480_000_000, 3),
    (1_070_000_000, 1_370_000_000, 3),
    (2_400_000_000, 2_480_000_000, 3),
    (5_725_000_000, 5_875_000_000, 3),
]

ignore_counter = {}
ignore_limit = 5
ignore_duration = 60

os.makedirs("logs", exist_ok=True)
log_filename = datetime.now().strftime("logs/log_%Y-%m-%d_%H-%M-%S.txt")

def setup_sdr():
    while True:
        try:
            print("🔌 Сканирование доступных устройств HackRF...")
            devices = SoapySDR.Device.enumerate({'driver': 'hackrf'})
            if len(devices) == 0:
                raise Exception("Устройства HackRF не найдены.")
            if SERIAL_NUMBER:
                print(f"🔌 Ищем устройство с серийным номером: {SERIAL_NUMBER}")
                device_found = None
                for dev in devices:
                    if 'serial' in dev and dev['serial'] == SERIAL_NUMBER:
                        device_found = dev
                        break
                if device_found is None:
                    raise Exception(f"Устройство с серийным номером {SERIAL_NUMBER} не найдено.")
                sdr = SoapySDR.Device(device_found)
                print(f"🔌 Устройство с серийным номером {SERIAL_NUMBER} найдено.")
            else:
                print("🔌 Подключаем первое найденное устройство HackRF.")
                sdr = SoapySDR.Device(devices[0])

            sdr.setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE)
            sdr.setGain(SOAPY_SDR_RX, 0, GAIN)

            hw = sdr.getHardwareInfo()
            serial = hw['serial'] if 'serial' in hw else 'unknown'
            print(f"Серийный номер подключенного HackRF: {serial}")
            return sdr
        except Exception as e:
            print(f"⚠️ Ошибка подключения к HackRF: {e}. Повторная попытка через 5 секунд...")
            time.sleep(5)

def measure_signal_power(sdr, center_freq):
    try:
        sdr.setFrequency(SOAPY_SDR_RX, 0, int(center_freq))
        rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        sdr.activateStream(rxStream)

        buff = np.empty(SAMPLES_PER_READ, np.complex64)
        sr = sdr.readStream(rxStream, [buff], SAMPLES_PER_READ, timeoutUs=1000000)

        sdr.deactivateStream(rxStream)
        sdr.closeStream(rxStream)

        if sr.ret > 0:
            data = buff[:sr.ret]
            fft = np.fft.fftshift(np.fft.fft(data))
            psd = 10 * np.log10(np.abs(fft) ** 2 + 1e-12)
            max_power = float(np.max(psd))
            print(f"  [MEASURE] {center_freq//1_000_000} MHz -> {max_power:.2f} dBFS")
            return max_power
        else:
            print(f"  [MEASURE] {center_freq//1_000_000} MHz -> нет данных (sr.ret={sr.ret})")
            return None
    except Exception as e:
        print(f"Ошибка measure_signal_power на {center_freq//1_000_000} MHz: {e}")
        return None

def live_listen(sdr, frequency, median_noise, duration=2, samples=10):
    print(f"\n▶️ Продолжаем прослушивание {frequency//1_000_000} MHz в течение ~{duration} секунд...")
    signal_above_threshold = 0
    for i in range(samples):
        power = measure_signal_power(sdr, frequency)
        if power is not None:
            print(f"  {frequency//1_000_000} MHz (sample {i+1}/{samples}): {power:.2f} dBFS")
            if power - median_noise > THRESHOLD_DELTA:
                signal_above_threshold += 1
        else:
            print(f"  {frequency//1_000_000} MHz: ошибка чтения")
        time.sleep(duration / samples)
    print(f"  [LISTEN RESULT] {frequency//1_000_000} MHz -> {signal_above_threshold}/{samples} раз выше Δ{THRESHOLD_DELTA}")
    return signal_above_threshold

def adai(pin=17):
    pwm = GPIO.PWM(pin, 440)
    melody = [
        (659, 0.18), (880, 0.12), (987, 0.18), (880, 0.12),
        (784, 0.18), (659, 0.18), (880, 0.25), (0, 0.12),
        (987, 0.18), (1046, 0.18), (987, 0.12), (880, 0.25),
        (784, 0.15), (880, 0.12), (659, 0.25)
    ]
    try:
        print("🎵 Исполняется 'Адай' (фрагмент)...")
        pwm.start(50)
        for freq, dur in melody:
            if freq == 0:
                pwm.ChangeDutyCycle(0)
            else:
                pwm.ChangeDutyCycle(50)
                pwm.ChangeFrequency(freq)
            time.sleep(dur)
        pwm.stop()
    finally:
        pass

def signal_it(fast=False):
    pwm = GPIO.PWM(pin, 2000)
    try:
        print(f"🔔 signal_it called, fast={fast}")
        repeats = 3
        for i in range(repeats):
            GPIO.output(pin_led, GPIO.HIGH)
            pwm.start(50)
            time.sleep(0.1 if fast else 0.25)
            pwm.ChangeDutyCycle(0)
            GPIO.output(pin_led, GPIO.LOW)
            time.sleep(0.05 if fast else 0.15)
            print(f"  signal pulse {i+1}/{repeats}")
    finally:
        pwm.stop()
        GPIO.output(pin_led, GPIO.LOW)

def signal_ignore():
    pwm = GPIO.PWM(pin, 2000)
    try:
        print(f"🔔 signal_ignore called, fast={fast}")
        repeats = 6
        for i in range(repeats):
            GPIO.output(pin_led, GPIO.HIGH)
            pwm.start(50)
            time.sleep(0.1)
            pwm.ChangeDutyCycle(0)
            GPIO.output(pin_led, GPIO.LOW)
            time.sleep(0.05)
            print(f"  signal pulse {i+1}/{repeats}")
    finally:
        pwm.stop()
        GPIO.output(pin_led, GPIO.LOW)

def try_find_shifted_freq(sdr, detected_freq, median_noise, dynamic_threshold, start_freq, end_freq, max_offsets=5):
    print(f"🔎 try_find_shifted_freq around {detected_freq//1_000_000} MHz")
    candidates = []
    steps_left = int((detected_freq - start_freq) // STEP)
    steps_right = int((end_freq - detected_freq) // STEP)
    max_possible_offsets = min(max_offsets, max(steps_left, steps_right))
    for offset in range(1, max_possible_offsets + 1):
        for direction in (+1, -1):
            new_freq = detected_freq + direction * offset * STEP
            if new_freq < start_freq or new_freq > end_freq:
                continue
            power = measure_signal_power(sdr, new_freq)
            if power is not None:
                print(f"  Проверка соседней частоты {new_freq//1_000_000} MHz: {power:.2f} dBFS")
                if power - median_noise > dynamic_threshold:
                    candidates.append((new_freq, power))
            else:
                print(f"  {new_freq//1_000_000} MHz: ошибка чтения при поиске смещённой частоты")
    if not candidates:
        print("  Нет кандидатов смещённых частот")
        return None
    candidates.sort(key=lambda x: x[1], reverse=True)
    best_freq = candidates[0][0]
    print(f"🔎 Найдена смещённая частота: {best_freq//1_000_000} MHz (силa {candidates[0][1]:.2f} dBFS)")
    return best_freq

def should_ignore(freq):
    global ignore_counter
    now = time.time()
    for f in list(ignore_counter.keys()):
        if now - ignore_counter[f]["time"] > ignore_duration:
            print(f"🧾 Сброс игнорирования для {f//1_000_000} MHz (время истекло).")
            del ignore_counter[f]
    if freq in ignore_counter and ignore_counter[freq]["count"] >= ignore_limit:
        print(f"⚠️ Частота {freq//1_000_000} MHz временно игнорируется (count={ignore_counter[freq]['count']}).")
        signal_ignore()
        return True
    return False

def mark_ignore(freq):
    global ignore_counter
    if freq not in ignore_counter:
        ignore_counter[freq] = {"count": 1, "time": time.time()}
    else:
        ignore_counter[freq]["count"] += 1
        ignore_counter[freq]["time"] = time.time()
    msg = f"🧾 Пометка частоты {freq//1_000_000} MHz как подозрительной (count={ignore_counter[freq]['count']})."
    print(msg)
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(log_filename, "a") as f:
        f.write(f"[{timestamp}] IGNORE {freq/1e6:.3f} MHz count={ignore_counter[freq]['count']}\n")

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
            print(f"{freq//1_000_000} MHz: {strength:.2f} dBFS")
            if len(readings) >= 3:
                median_noise = statistics.median(list(readings.values()))
                noise_spread = max(readings.values()) - min(readings.values())
                dynamic_threshold = BASE_THRESHOLD + min(noise_spread / 2, MAX_THRESHOLD - BASE_THRESHOLD)
                dynamic_threshold = round(dynamic_threshold, 2)

                print(f"  Порог шума {median_noise:.2f} dBFS, шумовой разброс {noise_spread:.2f}, динамический порог {dynamic_threshold:.2f}")
                if strength - median_noise > dynamic_threshold:
                    print(f"⚙️ Δ={strength - median_noise:.2f} > {dynamic_threshold:.2f} (возможно активность)")
                    signal_above_threshold = live_listen(sdr, freq, median_noise, LISTEN_DURATION)
                    print(f"📊 Повторное превышение порога: {signal_above_threshold} раз")
                    if signal_above_threshold >= threshold:
                        print(f"🚨 Обнаружено активное излучение на частоте {freq//1_000_000} MHz!")
                        if 2_300_000_000 <= freq <= 2_500_000_000:
                            print("  Частота в 2.4 или 5.8 диапазоне -> обычный сигнал")
                            signal_it(fast=True)
                        else:
                            print("  Частота не 2.4/5.8 -> быстрый сигнал и пометка/игнорирование при повторе")
                            signal_it(fast=False)
                            mark_ignore(freq)
                        tracked_freq = freq
                        repeat_count = 0
                        track_attempts = 0
                        while True:
                            if track_attempts >= 6 and not (2_300_000_000 <= freq <= 2_500_000_000):
                                print("⏹ Превышен лимит попыток трекинга вне диапазона 2.4/5.8 — выходим из трекинга.")
                                break
                            checks = 3
                            still_here = False
                            for _ in range(checks):
                                power_check = measure_signal_power(sdr, tracked_freq)
                                if power_check is None:
                                    print(f"❌ Ошибка повторного чтения {tracked_freq//1_000_000} MHz")
                                    break
                                diff = power_check - median_noise
                                print(f"🔁 Проверка {tracked_freq//1_000_000} MHz: {power_check:.2f} dBFS (Δ={diff:.2f})")
                                if diff > dynamic_threshold:
                                    still_here = True
                                    break
                                time.sleep(0.25)
                            if still_here:
                                repeat_count += 1
                                track_attempts += 1
                                print(f"📡 Сигнал всё ещё активен ({repeat_count} повторов)...")
                                signal_it(fast=False)
                                if repeat_count >= 3 and not (2_300_000_000 <= freq <= 2_500_000_000):
                                    print(f"🧾 Частота {tracked_freq//1_000_000} MHz уходит в игнор на {ignore_duration} сек (после {repeat_count} повторов).")
                                    mark_ignore(tracked_freq)
                                    break
                                time.sleep(1)
                                continue
                            new_freq = try_find_shifted_freq(sdr, tracked_freq, median_noise, dynamic_threshold, start_freq, end_freq, max_offsets=5)
                            if new_freq:
                                tracked_freq = new_freq
                                track_attempts += 1
                                print(f"🔁 Переключаемся на новую частоту {tracked_freq//1_000_000} MHz и продолжаем трекинг.")
                                signal_it(fast=False)
                                time.sleep(1)
                                continue
                            else:
                                print("✅ Сигнал исчез в пределах сканируемого окна.")
                                break
        else:
            print(f"{freq//1_000_000} MHz: ошибка чтения сигнала")
        freq += STEP
        time.sleep(0.15)
    return readings

def main():
    try:
        sdr = setup_sdr()
        adai()
        while True:
            for start, end, threshold in FREQUENCY_RANGES:
                scan_frequency_range(sdr, start, end, threshold)
                print("\n\n\n\n\n")
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\n🛑 Завершено пользователем.")
    except Exception as e:
        GPIO.cleanup()
        print(f"\n🛑 Ошибка в main: {e}")
        raise

if __name__ == '__main__':
    main()
