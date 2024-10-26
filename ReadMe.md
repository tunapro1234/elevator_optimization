# Yiğit bunu senin için yazıyorum burayı güncellememiz lazım
Simülasyonu olabildiğince gerçeğe yakın kılmak için hem makine kısmında hem de popülasyon kısmında iyi mekanizmalarımız olmalı 

# Geleneksel Asansör Algoritmaları
Selam Tuna'cım, yaygın olarak kullanılan iki algoritmayı belirledim:
# Elevator Management Algorithms in Python

## 1. Nearest Car Algorithm

Biri asansör çağırdığında **idle** veya çağrının yapıldığı kata doğru ilerleyen ve aynı zamanda çağrının yapıldığı yöne ilerleyen (eğer asansörde kullanıcı yukarı veya aşağı gitmek istediğini önceden belirtiyorsa, ki bu günümüz asansörlerinde çok yaygın) asansör gönderilir. Ardından, asansör sırayla kullanıcıların seçtiği yönde gidilmek istenilen katlara bırakarak ilerler.

### Python Kod Örneği

```python
class Elevator:
    def __init__(self, id, current_floor, direction, idle=True):
        self.id = id
        self.current_floor = current_floor
        self.direction = direction  # 'up', 'down', or 'idle'
        self.idle = idle

    def move_to_floor(self, target_floor):
        self.current_floor = target_floor
        print(f"Elevator {self.id} moved to floor {target_floor}")

class NearestCarAlgorithm:
    def __init__(self, elevators):
        self.elevators = elevators

    def find_nearest_elevator(self, call_floor, call_direction):
        nearest_elevator = None
        min_distance = float('inf')
        
        for elevator in self.elevators:
            if elevator.idle or elevator.direction == call_direction:
                distance = abs(elevator.current_floor - call_floor)
                if distance < min_distance:
                    nearest_elevator = elevator
                    min_distance = distance
        
        return nearest_elevator

    def handle_call(self, call_floor, call_direction):
        elevator = self.find_nearest_elevator(call_floor, call_direction)
        if elevator:
            elevator.idle = False
            elevator.direction = call_direction
            elevator.move_to_floor(call_floor)
        else:
            print("No available elevator to handle the call.")

# Örnek kullanım
elevators = [Elevator(1, 0, 'idle'), Elevator(2, 5, 'up'), Elevator(3, 10, 'idle')]
algorithm = NearestCarAlgorithm(elevators)

# Bir çağrı örneği
algorithm.handle_call(3, 'up')
```
## 2. Simple Collective Control Algorithm

Biri asansör çağırdığında çağrının yapıldığı yöne doğru ilerleyen ve o yönde devam eden bir asansör varsa, bu asansör çağrıya atanır. Asansör, hareket ettiği yönde diğer çağrıları da sırayla alır ve o yöne gitmek isteyen kullanıcıları katlarına bırakır. Tüm çağrılar tamamlanınca asansör yön değiştirir ve bu kez diğer yönde gelen çağrılara yanıt verir.

### Python Kod Örneği
```python
class Elevator:
    def __init__(self, id, current_floor, direction='idle', idle=True):
        self.id = id
        self.current_floor = current_floor
        self.direction = direction  # 'up', 'down', or 'idle'
        self.idle = idle
        self.stops = []

    def add_stop(self, floor):
        if floor not in self.stops:
            self.stops.append(floor)
            self.stops.sort(reverse=self.direction == 'down')

    def move(self):
        if self.stops:
            next_stop = self.stops.pop(0)
            print(f"Elevator {self.id} moving to floor {next_stop}")
            self.current_floor = next_stop
            if not self.stops:
                self.direction = 'idle'
                self.idle = True
        else:
            print(f"Elevator {self.id} is idle at floor {self.current_floor}")

class SimpleCollectiveControl:
    def __init__(self, elevators):
        self.elevators = elevators

    def assign_call(self, call_floor, call_direction):
        for elevator in self.elevators:
            if elevator.idle or elevator.direction == call_direction:
                elevator.direction = call_direction
                elevator.idle = False
                elevator.add_stop(call_floor)
                break
        else:
            print("No available elevator to handle the call.")

    def step(self):
        for elevator in self.elevators:
            elevator.move()

# Örnek kullanım
elevators = [Elevator(1, 0), Elevator(2, 5), Elevator(3, 10)]
control_system = SimpleCollectiveControl(elevators)

# Çağrıları sisteme atanma ve adım adım hareket simülasyonu
control_system.assign_call(3, 'up')
control_system.assign_call(7, 'up')
control_system.assign_call(2, 'down')

# Adımları simüle etme
for _ in range(5):
    control_system.step()
```

# Makine kısmı 
Bu kısım makine tarafında yazdığımız kodların temel özelliklerini içeriyor
Makine kısmında motor, elevator ve elevator system olmak üzere 3 temel yapı var.
3 temel yapı için de konfigürasyon yapmayı sağlayan 3 tane yaml config dosyası olacak, 
konfigürasyon detaylarını başlıkların altında vereceğim

## PID Controller
PID kontrol mekanizması elevator ve motor yapılarımızda kullanılıyor. Motorda belirli 
bir hıza gelmek için verilmesi gerek akımı hesaplıyor, elevatorda ise belirli bir 
yüksekliğe gelmek için gereken hızı hesaplıyor.

pid kontrolcüsünün parametrelerini içeren bir dosya aşağıda örnek olarak var.
```yaml
    kp: 1.
    ki: 0.
    kd: 0.
    integral_limit: 0.

    update_freq: 30.

    enable_target_limits: true
    max_target: 200.
    min_target: -200.

    enable_output_limits: true
    max_output: 20.
    min_output: -20.

    #   Change Limit değişkeni outputun bir 
    # saniyede ne kadar değişebileceğini belirliyor
    change_limit: 1.
    # Akım artış azalışını kısıtlayarak ivmeyi sınırlandırabiliyoruz
    #   Onun dışında, çalışma frekansı yerine saniyeye bağlı olduğu 
    # için pid frekansı değiştiğine bu değeri değiştirmemiz gerekmiyor

    enable_debug_plotting: true
```
`kp`: pid p katsayısı
`ki`: pid i katsayısı
`kd`: pid d katsayısı
`integral_limit`: pid integral limiti
`update_freq`: döngü frekansı, eğer geçmesi gereken süre geçmeden çağrılırsa eski sonucu döndürüyor. Devre dışı bırakmak için 0 verilebilir.
`enable_target_limits`: pid girdi kısıtlaması aktifleştirilsin mi
`max_target`: açıklayıcı
`min_target`: bu da
`enable_output_limits`: pid çıktısını kısıtlamak için limitler aktifleştirilsin mi
`max_output`: açıklayıcı
`min_output`: bu da

`change_limit`: pidnin çok hızlı çıktı değiştirmesini önlemek için
`enable_debug_plotting`: pid objesi için plotting açık olacak mı, debuggingde kullanılıyor

## Motor
Motor kısmında elevator ve elevator systemde olduğu gibi yaml config dosyası var. Bunun yanı sıra
motorun hangi akımda hangi özelliklere sahip olduğunu gösteren bir tane de sample dosyası var.

Motor içinde hızı ayarlamak için pid algoritması var, bu pid algoritması şu anki hızımızla hedef 
hız arasındaki farktan vermek istediğimiz akımı belirliyor, bu verdiğimiz akımı kullanarak hem 
harcadığımız enerjiyi ölçüyoruz hem de yeni hızımızı hesaplıyoruz

Motor sample dosyası verebileceğimiz tüm akım aralığının datasını içeremeyeceği için aradaki boşlukları 
en yakın iki nokta arasında çizgi çekip noktayı xteki input olarak kullanıp buluyor.
buna interpolation deniyomuş yeni öğrendim ben manuel hesaplamıştım ama yapması için bi kütüphane 
olabilir bakmak lazım

Motor classımızda gearbox ratio var ve bu dışarıdan hız hesaplarken ve dışarıya hız çıktısı verirken 
hesaplamaya dahil ediliyor (motor şaftının rpminden elevator hızı hesaplamak için). Bunun yanı sıra 
dış kasnağın çevresini de kullanıyoruz. 

Motor objeleri toplamda ne kadar enerji harcadıklarını kaydediyor

Motorlar hard ve soft akım ve rpm sınırlandırması yapıyor. Hard sınırlandırma sample dosyasındaki 
max değerlere göre belirleniyor. Soft ise motorun parametreler dosyasında belirleniyor. Her zaman 
küçük olan değer gerçek sınır olarak alınıyor ve o sınır geçilmiyor.

Örnek bir motor config dosyası:
```yaml
pid_parameters:
    # Motorun hız pidsi için gerekli parametreler buraya geliyor

gearbox_ratio: 1.
output_shaft_radius: 10.
sample_path: "data/motor_samples.csv"
soft_rpm_limit: 200.
soft_current_limit: 50.
```

`gearbox_ratio`: gearbox oranı
`output_shaft_radius`: çıkış şaft yarıçapı, asansör hızını hesaplarken kullanılıyor
`sample_path`: motor sample dosyasının pathi
`soft_rpm_limit`: yeteri kadar açıklayıcı
`soft_current_limit`: bu da

## Elevator
Elevator kısmında da parametreleri içeren bir yaml dosyası olacak. Bunu yazmaya devam ediyorum, 
bitirince açıklamasını da tamamlicam. Şimdilik kat yüksekliklerinin farklı olabileceğini yazsam 
yeterli olur herhalde.
