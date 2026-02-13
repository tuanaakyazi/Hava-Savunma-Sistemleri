# Hava-Savunma-Sistemleri

Proje Hakkında

Bu proje, gerçek zamanlı ya da kaydedilmiş görüntüler üzerinden hava hedeflerini algılamak ve sınıflandırmak için Python dilinde üç aşamalı modüller (asama1.py, asama2.py, asama3.py) içerir.
Projenin genel amacları:
Görüntü tabanlı hedef tespit ve sınıflandırma
Modüler mimari ile farklı aşama deneyleri
Eğitim / araştırma prototipi olarak kullanılabilir

AŞAMA1
Sistemin temel kontrol mekanizması oluşturulmuştur. OpenCV ile gerçek zamanlı görüntü işleme yapılarak kırmızı (düşman) ve mavi (dost) hedefler ayırt edilir. Tespit edilen hedefler, step ve servo motorlar aracılığıyla otonom olarak takip edilerek ekranın merkezine kilitlenir.

AŞAMA2
Takip algoritması dinamik hata payı ($dx, dy$) hesaplamalarıyla optimize edilmiştir. Hedef belirlenen "güvenli bölgede" sabitlendiğinde, sistem otomatik bir geri sayım başlatır ve belirlenen süre sonunda imha mekanizmasını (trigger) otonom olarak tetikler.

AŞAMA3
Sisteme stratejik karar verme yeteneği kazandırılmıştır. pyzbar entegrasyonu ile QR kodları (A/B komutları) okunarak göreve özel yönelimler gerçekleştirilir. Ayrıca approxPolyDP ile geometrik şekil analizi yapılarak, sistemin sadece belirli (öğretilmiş) hedeflere angajman sağlaması gerçekleştirilmiştir.
