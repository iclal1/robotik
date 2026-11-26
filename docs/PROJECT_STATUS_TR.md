# Proje durumu (26 Kasım 2026)

Bu belge, raporlarla doğrulanmış geliştirmeyi özetler. Gazebo'da hareket gözlemi, yalnızca plan üretimi ve gelecekteki hedefler ayrı değerlendirilir. Hiçbir küpün otonom olarak hedefe teslim edildiği gösterilmedi.

| Aşama | Tamamlanan ve doğrulanan iş | Sınır |
| --- | --- | --- |
| 1–2 | Gazebo Fortress yarışma sahası, diferansiyel robot, pasif itici, RGB kamera, LiDAR, ROS köprüleri, odometri ve TF kuruldu. Sensör yayınları ve sınırlı manuel/deneysel hareket doğrulandı. | Bu deneyler otonom teslimat değildir. |
| 3–4 | Kamera renk/küp algısı, geometri, güvenlik ve erken keşif bileşenleri geliştirildi. | Üretim düzeyinde üç küp görevi kanıtlanmadı. |
| 5B | SLAM Toolbox ile yarışma haritası kaydedildi. 119×79, 0,05 m gridde 8519 serbest, 607 dolu, 275 bilinmeyen hücre; bilinen alan yaklaşık %97,07. Map Server ve Nav2 entegrasyonu yapıldı. | Harita değiştirilmedi; bilinmeyen hücreler korunmalı. |
| 5C | AMCL'nin sıfır kovaryanslı başlangıç hatası giderildi; harita, tarama, odometri ve yaşam döngüsü hazır olduktan sonra kovaryanslı ilk poz verildi. | Başlangıç varsayımı ölçülmüş mutlak doğruluk değildir. |
| 5D | Kısa, sınırlandırılmış Nav2 öteleme ve dönme denemeleri; güvenlik gözetimi ve küp konumları izlendi. | Saha geçişi yok. |
| 5E | Hareket sırasında dinamik AMCL güncellemesi gözlendi. Bir deneme, ölçüm aracının yanlış sensör tazeliği alarmıyla iptal edildi; acil durdurma çalıştı. | İptal edilen eylem başarı sayılmadı. |
| 5E-2 | Ölçüm aracı düzeltildi. Üç sınırlandırılmış Nav2 denemesi ve hareket sırasında AMCL güncellemesi doğrulandı; küpler hareket etmedi. | Uzun ve dar geçit hareketi yok. |
| 5F | Gazebo'da yaklaşık 0,335 m gerçek hareket, Nav2 `SUCCEEDED`, göreli hedef hatası yaklaşık 0,0936 m, küp yer değiştirmesi yok. | Daha uzun yol planlaması o oturumda başarısız oldu. |
| 5F-2 | Uzun yol planlama engelinin 2,58 m belirsizliğe genişleyen yeşil küp iziyle bağlantısı teşhis edildi. | Engeli silmek güvenli çözüm değildi. |
| 5G | Tekrarlı kısmi gözlemlerde belirsizlik büyümesi düzeltildi: aynı teşhis senaryosunda yaklaşık 2,53 m → 0,42 m. Taze ölçümle maliyet haritası engelinin kanıta dayalı güncellenmesi ve açık alanda plan üretimi doğrulandı. 70 test geçti. | Dar geçit rotaları eski disk denetiminde başarısızdı. |
| 5H | İtici ve tekerlekleri içeren yönlü çokgen ayak izi denetimi eklendi. B geçidi iki yönde, mevcut iki sigma yön belirsizliği altında planlama denetiminden geçti. A geçidi nominal olarak sığsa da belirsizlik marjıyla başarısız oldu. 76 test geçti. | Geçitlerde fiziksel sürüş yapılmadı. |

Robotun tam düzlemsel ayak izi 0,4375×0,335 m'dir. Haritada A/B açıklıkları yaklaşık 1,05/1,15 m; fiziksel sahada 1,10/1,20 m'dir. 5H'nin harita için istediği açıklık 0,31827 m idi. A'daki ölçülmüş yön belirsizliği ve mevcut başlangıçtan yaklaşım, güvenli rota iddiasını engelliyor. B'nin iki yönlü sonucu yalnızca planlama ve bağımsız ayak izi hesabıdır; sürüş sonucu değildir.

Kaynaklar: [5B](../reports/phase5b_final_status.md), [5C](../reports/phase5c_amcl_localization_validation.md), [5D](../reports/phase5d_dynamic_navigation_validation.md), [5E](../reports/phase5e_dynamic_amcl_navigation.md), [5E-2](../reports/phase5e2_sensor_freshness_navigation_validation.md), [5F](../reports/phase5f_extended_navigation_validation.md), [5F-2](../reports/phase5f2_long_distance_planning.md), [5G](../reports/phase5g_cube_tracking_costmap_validation.md), [5H](../reports/phase5h_narrow_passage_validation.md). Önceki deneyler için [Phase 2](phase2_validation.md) ve [Phase 4](../reports/phase4_engineering_report.md).
