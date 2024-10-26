# Yiğit bunu senin için yazıyorum burayı güncellememiz lazım
Simülasyonu olabildiğince gerçeğe yakın kılmak için hem makine kısmında hem de popülasyon kısmında iyi mekanizmalarımız olmalı 

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
