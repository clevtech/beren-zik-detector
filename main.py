import SoapySDR
from SoapySDR import *  # SOAPY_SDR_ constants
import numpy as np
import time
import statistics
import os
import requests
from datetime import datetime
import RPi.GPIO as GPIO


pin = 17
pin_led = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.setup(pin_led, GPIO.OUT, initial=GPIO.LOW)


# Параметры HackRF
SAMPLE_RATE = 10000000
GAIN = 100
SAMPLES_PER_READ = 8192
SERIAL_NUMBER = None

# Параметры сканирования
# === Порог обнаружения (адаптивный режим) ===
BASE_THRESHOLD = 2      # минимальный порог в dB (при чистом эфире)
MAX_THRESHOLD = 10      # максимальный порог
ADAPTIVE = True         # включаем адаптацию
THRESHOLD_DELTA = 3

STEP = 10_000_000
LISTEN_DURATION = 2

# Частотные диапазоны (центр, разброс)
FREQUENCY_RANGES = [
    (902_000_000, 928_000_000, 3),
    (2_400_000_000, 2_480_000_000, 3),
    (1_070_000_000, 1_370_000_000, 3),
    (2_400_000_000, 2_480_000_000, 3),
    (5_725_000_000, 5_875_000_000, 3),
]


os.makedirs("logs", exist_ok=True)
log_filename = datetime.now().strftime("logs/log_%Y-%m-%d_%H-%M-%S.txt")

def setup_sdr():
    while True:
        try:
            
            print("🔌 Сканирование доступных устройств HackRF...")
            # Получаем список всех доступных устройств
            devices = SoapySDR.Device.enumerate({'driver': 'hackrf'})
            
            if len(devices) == 0:
                raise Exception("Устройства HackRF не найдены.")
            
            if SERIAL_NUMBER:
                print(f"🔌 Ищем устройство с серийным номером: {SERIAL_NUMBER}")
                device_found = None
                for dev in devices:
                    if dev['serial'] == SERIAL_NUMBER:
                        device_found = dev
                        break
                if device_found is None:
                    raise Exception(f"Устройство с серийным номером {SERIAL_NUMBER} не найдено.")
                print(f"🔌 Устройство с серийным номером {SERIAL_NUMBER} найдено.")
                sdr = SoapySDR.Device(device_found)
            else:
                
                print("🔌 Подключаем первое найденное устройство HackRF.")
                
                sdr = SoapySDR.Device(devices[0])
            
            # Пытаемся установить соединение
            sdr.setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE)
            sdr.setGain(SOAPY_SDR_RX, 0, GAIN)

            # Проверка соединения с устройством
            
            print(f"Серийный номер подключенного HackRF: {sdr.getHardwareInfo()['serial']}")
            return sdr
        except Exception as e:
            
            print(f"⚠️ Ошибка подключения к HackRF: {e}. Повторная попытка через 5 секунд...")
            time.sleep(5)

def measure_signal_power(sdr, center_freq):
    sdr.setFrequency(SOAPY_SDR_RX, 0, center_freq)
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.activateStream(rxStream)

    buff = np.empty(SAMPLES_PER_READ, np.complex64)
    sr = sdr.readStream(rxStream, [buff], len(buff), timeoutUs=1000000)
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)

    if sr.ret > 0:
        # FFT внутри 10 МГц пакета
        data = buff[:sr.ret]
        fft = np.fft.fftshift(np.fft.fft(data))
        psd = 10 * np.log10(np.abs(fft)**2 + 1e-12)

        # Возвращаем максимум в этом спектре
        max_power = np.max(psd)
        return max_power
    else:
        return None

def live_listen(sdr, frequency, median_noise, duration=2):
    print(f"\n▶️ Продолжаем прослушивание {frequency//1_000_000} MHz в течение {duration} секунд...")
    end_time = time.time() + duration
    signal_above_threshold = 0
    for _ in range(10):
        
        power = measure_signal_power(sdr, frequency)
        if power is not None:
            print(f"  {frequency//1_000_000} MHz: {power:.2f} dBFS")
            if power - median_noise > THRESHOLD_DELTA:  # <-- фикс тут
                signal_above_threshold += 1
        else:
            print(f"  {frequency//1_000_000} MHz: ошибка чтения")
        time.sleep(0.2)
    return signal_above_threshold

def adai(pin=17):
    """Исполняет короткий фрагмент кюя 'Адай' на пассивном буззере."""
    pwm = GPIO.PWM(pin, 440)  # стартовая частота

    # Упрощённый музыкальный фрагмент "Адай" — динамичный и узнаваемый ритм
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
                pwm.ChangeDutyCycle(0)  # пауза
            else:
                pwm.ChangeDutyCycle(50)
                pwm.ChangeFrequency(freq)
            time.sleep(dur)
        pwm.stop()
    finally:
        pass

def signal_it():
    print("Сигнал!!!")
    pwm = GPIO.PWM(pin, 2000)  # частота для буззера (2 кГц)

    try:
        for _ in range(3):
            GPIO.output(pin_led, GPIO.HIGH)  # включить LED
            pwm.start(50)                    # 50% duty cycle (звук)
            time.sleep(0.25)                 # длительность писка

            pwm.ChangeDutyCycle(0)           # отключить звук, не останавливая PWM
            GPIO.output(pin_led, GPIO.LOW)   # выключить LED
            time.sleep(0.15)                 # пауза между писками
    finally:
        pwm.stop()
        GPIO.output(pin_led, GPIO.LOW)


def scan_frequency_range(sdr, center_freq, spread, threshold):
    start_freq = center_freq
    end_freq = spread
    readings = {}

    print(f"\nСканирование диапазона {start_freq//1_000_000}-{end_freq//1_000_000} MHz...")

    for freq in range(start_freq, end_freq + 1, STEP):
        strength = measure_signal_power(sdr, freq)
        if strength is not None:
            readings[freq] = strength
            print(f"{freq//1_000_000} MHz: {strength:.2f} dBFS")

            if len(readings) >= 3:
                median_noise = statistics.median(list(readings.values()))

                # === АДАПТИВНЫЙ ПОРОГ ===
                noise_spread = max(readings.values()) - min(readings.values())
                dynamic_threshold = BASE_THRESHOLD + min(noise_spread / 2, MAX_THRESHOLD - BASE_THRESHOLD)
                dynamic_threshold = round(dynamic_threshold, 2)

                # === Проверка на превышение ===
                if strength - median_noise > dynamic_threshold:
                    print(f"⚙️ Порог шума {median_noise:.2f} dBFS, Δ={strength - median_noise:.2f} > {dynamic_threshold:.2f}")
                    signal_above_threshold = live_listen(sdr, freq, median_noise, LISTEN_DURATION)
                    print(f"📊 Повторное превышение порога: {signal_above_threshold} раз")

                    if signal_above_threshold >= threshold:
                        print(f"🚨 Наличие дрона на частоте {freq//1_000_000} MHz!")
                        signal_it()

                        # === Цикл до исчезновения сигнала ===
                        while True:
                            power_check = measure_signal_power(sdr, freq)
                            if power_check is None:
                                print(f"❌ Ошибка повторного чтения {freq//1_000_000} MHz")
                                break

                            diff = power_check - median_noise
                            print(f"🔁 Проверка {freq//1_000_000} MHz: {power_check:.2f} dBFS (Δ={diff:.2f})")

                            if diff > dynamic_threshold:
                                print("📡 Дрон всё ещё активен...")
                                signal_it()
                                time.sleep(1)
                            else:
                                print("✅ Сигнал дрона исчез.")
                                break
                        
                        # После исчезновения сигнала — продолжаем сканирование
        else:
            print(f"{freq//1_000_000} MHz: ошибка чтения сигнала")

        time.sleep(0.2)

    return readings


def log_detection(freq, level):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    log_line = f"[{timestamp}] Обнаружен дрон на частоте: {freq / 1e9:.3f} GHz. {level:.2f} dBFS.\n"
    # with open(log_filename, "a") as f:
    #     f.write(log_line)

def main():
        
    try:        
        

        sdr = setup_sdr()

        adai()
        
        while True:
            for center, spread, threshold in FREQUENCY_RANGES:
                readings = scan_frequency_range(sdr, center, spread, threshold)

            print("\n\n\n\n\n")
    except:
        GPIO.cleanup()
        print("\n🛑 Завершено пользователем.")
        raise

if __name__ == '__main__':
    main()
