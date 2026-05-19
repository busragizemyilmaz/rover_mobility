# Rover Mobility ve Odometri

ROS 2 tabanlı rover hareket sistemi. Joystick girdisini işler, STM32 mikrodenetleyicisine UART üzerinden komut gönderir ve encoder geri bildirimini okuyarak tekerlek mesafelerini hesaplar.

---

## Mimari

```
[Joystick]
    │
    ▼
[drive_teleop]  ── /wheel_speeds ──►  [drive_hardware]  ── UART ──►  [STM32]
                                                                          │
[rover_encoder] ◄── /encoder_raw ────────────────────────────────────────┘
    │
    ▼
/wheel_distances
```

---

## Paketler

```
rover_mobility/
├── drive_teleop/                         ← Joystick → /wheel_speeds (Python)
│   └── drive_teleop/
│       └── tank_drive_joystick.py
│   └── launch/teleop.launch.py
│
├── drive_hardware/                       ← /wheel_speeds → STM32 UART (C++)
│   └── src/rover_serial_driver.cpp
│   └── launch/hardware.launch.py
│
└── rover_encoder/                        ← /encoder_raw → /wheel_distances (Python)
    └── rover_encoder/encoder_dashboard.py
    └── fake_stm32.py                     ← Test aracı (gerçek STM32 olmadan)
```

| Paket | Dil | Görev |
|---|---|---|
| `drive_teleop` | Python | Joystick eksenlerini normalize edilmiş tekerlek hızlarına çevirir, `/wheel_speeds` yayınlar |
| `drive_hardware` | C++ | `/wheel_speeds` alır, 6 byte'lık paketi UART ile STM32'ye gönderir; encoder verisi okur |
| `rover_encoder` | Python | Ham encoder tiklerini metreye çevirir, terminal dashboard gösterir |

---

## Kurulum

```bash
# Workspace'e kopyala
cp -r rover_mobility ~/ros2_ws/src/

# Bağımlılıkları yükle
sudo apt install ros-$ROS_DISTRO-joy

# Derle
cd ~/ros2_ws
colcon build --packages-select drive_teleop drive_hardware rover_encoder

# Ortamı yükle (her yeni terminalde)
source install/setup.bash
```

---

## STM32 Bağlantısı

STM32, USB üzerinden `/dev/ttyACM0` portunda görünür. Her yeniden bağlantıda:

```bash
sudo chmod 666 /dev/ttyACM0
```

Port farklıysa bul:
```bash
ls /dev/ttyACM*
```

---

## Çalıştırma

Her paket kendi launch dosyasıyla ayrı ayrı başlatılır.

### 1. Joystick + Teleop

```bash
# Varsayılan: Tank Drive, sol eksen = axes[1], sağ eksen = axes[4], R1 aktivasyon butonu = 5
ros2 launch drive_teleop teleop.launch.py

# Joystick cihazını açıkça belirt (aynı bilgisayarda arm joystick de varsa)
ros2 launch drive_teleop teleop.launch.py joy_device:=/dev/input/js0

# Diferansiyel sürüş moduyla
ros2 launch drive_teleop teleop.launch.py joy_device:=/dev/input/js0 drive_mode:=2

# Joystick eksen ve buton indekslerini özelleştirerek
ros2 launch drive_teleop teleop.launch.py joy_device:=/dev/input/js0 left_axis_index:=1 right_axis_index:=3 activation_button_index:=5
```

> **Güvenlik:** Rover yalnızca **R1 butonuna (varsayılan: buton 5) basılı tutulduğu sürece** hareket eder.
> Bırakıldığında hedef hız sıfırlanır, ivmeleme sınırlayıcı yumuşak bir şekilde durdurur.
> Farklı bir joystick kullanıyorsan `activation_button_index` parametresini doğru butona ayarla.

> `joy_device` parametresi aynı bilgisayarda arm joystick de takılıysa
> **mutlaka açıkça belirtilmelidir.** Mobility joystick için `/mobility_joy`,
> arm joystick için `/robotarm_joy` topic'i kullanılır — topic çakışması yaşanmaz,
> ancak `joy_node`'un doğru fiziksel cihaza bağlandığından emin olmak için
> `joy_device` her zaman yazılmalıdır.

### 2. STM32 Sürücüsü (Jetson'da)

```bash
# Varsayılan: /dev/ttyACM0, 115200 baud
ros2 launch drive_hardware hardware.launch.py

# Port veya baud değiştirerek
ros2 launch drive_hardware hardware.launch.py port_name:=/dev/ttyACM1 baud_rate:=57600
```

### 3. Encoder Dashboard

```bash
ros2 run rover_encoder encoder_dashboard
```

Terminal ekrana canlı olarak her tekerlerin ham tik ve gidilen mesafe bilgisini yazar.

---

## Kontrol Modları

### Sürüş Modu

| `drive_mode` | Açıklama |
|---|---|
| `1` (Tank Drive) | Sol joystick ekseni sol tekerlekleri, sağ ekseni sağ tekerlekleri kontrol eder |
| `2` (Diferansiyel) | Sol eksen = ileri/geri hız, sağ eksen (`right_axis_index_diff`) = dönüş yönü |

### PWM / PID Geçişi

Joystick üzerindeki **mode(X) butonuna** (varsayılan: buton 0) basarak çalışma sırasında PWM ve PID modları arasında geçiş yapılabilir. Seçilen mod `/wheel_speeds` paketinin ilk elemanına kodlanır ve STM32'ye iletilir.

---

## STM32 Haberleşme Protokolü

### TX (ROS → STM32) — 6 Byte Paket

| Byte | İçerik | Değer |
|---|---|---|
| `[0]` | Header | `0xFF` |
| `[1]` | Kontrol modu | `0` = PWM, `1` = PID |
| `[2]` | Ön Sol hız | `0–255` (`127` = dur) |
| `[3]` | Arka Sol hız | `0–255` |
| `[4]` | Ön Sağ hız | `0–255` |
| `[5]` | Arka Sağ hız | `0–255` |

Normalize hız değerleri `[-1, 1]` aralığından `[0, 255]`'e dönüştürülür:
- `-1 → 0`, `0 → 127`, `+1 → 255`

### RX (STM32 → ROS) — 20 Byte Encoder Paketi

| Byte | İçerik |
|---|---|
| `[0–1]` | Header `0xBA 0xAB` |
| `[2–5]` | Encoder 1 (32-bit, big-endian) |
| `[6–9]` | Encoder 2 |
| `[10–11]` | Header `0xCD 0xDC` |
| `[12–15]` | Encoder 3 |
| `[16–19]` | Encoder 4 |

---

## Encoder Ayarları

`rover_encoder/encoder_dashboard.py` içindeki fiziksel parametreler:

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `TICKS_PER_REV` | `409600` | 1 tam tur = kaç tik |
| `WHEEL_RADIUS_M` | `0.10` m | Tekerlek yarıçapı |

Bu değerleri donanımına göre güncellemeyi unutma.

---

## Tüm Launch Parametreleri

### `drive_teleop` — `teleop.launch.py`

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `joy_device` | *(boş)* | Joystick cihaz yolu (ör: `/dev/input/js0`) — aynı bilgisayarda iki joystick varsa **mutlaka yaz** |
| `left_axis_index` | `1` | Sol tekerlek ekseni / linear eksen (Tank & Diff) |
| `right_axis_index` | `4` | Sağ tekerlek ekseni — Tank Drive |
| `right_axis_index_diff` | `3` | Angular (dönüş) ekseni — yalnızca Diferansiyel Drive modunda kullanılır |
| `drive_mode` | `1` | `1` = Tank Drive, `2` = Diferansiyel |
| `activation_button_index` | `5` | Dead-man butonu — PS4/PS5 = 5 (R1), Xbox = 5 (RB). Farklı joystick için değiştir. |

### `drive_hardware` — `hardware.launch.py`

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `port_name` | `/dev/ttyACM0` | STM32 seri port |
| `baud_rate` | `115200` | Baud hızı |
| `timeout_sec` | `0.5` | Bu süre içinde komut gelmezse rover durur |

---

## Geliştirme: Sahte STM32

Gerçek donanım olmadan encoder verisini test etmek için:

```bash
ros2 run rover_encoder fake_stm32
```

Sağ tekerleklere 500, sol tekerleklere 200 tik/mesaj gönderir — rover sola dönüyor gibi simüle eder. `encoder_dashboard` ile birlikte kullanılabilir.

---

## Sık Karşılaşılan Sorunlar

**STM32 portu açılamıyor:**
```bash
sudo chmod 666 /dev/ttyACM0
ls /dev/ttyACM*
```

**Joystick bulunamıyor:**
```bash
ls /dev/input/js*
ros2 topic echo /mobility_joy
```

**R1'e basıyorum ama rover hareket etmiyor:**

`activation_button_index` yanlış olabilir. Joystick buton haritanı kontrol et:
```bash
ros2 topic echo /mobility_joy
```
`buttons` dizisinde R1'e bastığında hangi index `1` oluyor? Ardından:
```bash
ros2 launch drive_teleop teleop.launch.py activation_button_index:=<doğru_index>
```

**Rover ters yönde gidiyor:**
`rover_serial_driver.cpp` içindeki `sendPacket()` fonksiyonunda ilgili tekerleğin hız değerini negatifle ya da byte atamalarını (`packet[2–5]`) değiştir.

**Komut gönderiliyor ama hareket yok:**
```bash
ros2 topic echo /wheel_speeds
ros2 topic echo /encoder_raw
```

**Yanlış joystick'i alıyor (arm joystick de aynı bilgisayarda):**
```bash
ls /dev/input/js*
ros2 launch drive_teleop teleop.launch.py joy_device:=/dev/input/js0
```

---

**RTPS_TRANSPORT_SHM hataları:**

```
[RTPS_TRANSPORT_SHM Error] Failed init_port ...
```

Bu hatalar zararsızdır, node'lar çalışmaya devam eder. Görmek istemiyorsan:
```bash
sudo rm -rf /dev/shm/fastrtps_*
```

---

## Bağlantılı Proje

Robotik kol kontrolü → [`robotic_arm`](https://github.com/busragizemyilmaz/robotic_arm)