# Ekip çalışma kılavuzu

## Başlangıç ve dal düzeni

GitHub adresi sağlandığında `git clone <repo-url> robotics_competition_ws`, ardından [kurulum kılavuzunu](SETUP_TR.md) izleyin. `main` dalını güncel tutun. Her iş için `git switch -c feature/<kisa-konu>` veya `git switch -c fix/<kisa-konu>` açın; PR ile amaç, değişen dosyalar, test komutları, ölçümler ve riskleri yazın. İnceleme sonrası birleştirin. Başkasının dalını zorla sıfırlamayın; çakışmada iki değişikliği de inceleyin.

## Birlikte geliştirme

Navigasyon, algı ve küp itme çalışmalarını ayrı dallarda yürütün. Ortak arayüzleri ve çerçeveleri değiştirmeden önce diğer alanlarla koordine edin. `src/competition_simulation/worlds/competition.sdf`, robot Xacro, sensör konumları, `src/competition_bringup/config/nav2_saved_map.yaml`, güvenlik gözetmeni, hız koruyucusu, acil durdurma ve kayıtlı harita hassastır. Bu dosyaların değişimi ölçülmüş gerekçe, güvenlik incelemesi ve geriye dönük test ister. Özellikle özgün SLAM YAML/PGM, 5B–5H karşılaştırmalarının temelidir; yeniden üretmeyin veya üzerine yazmayın.

PR öncesi `colcon build --symlink-install` ve `python3 -m pytest -q src/competition_bringup/test` çalıştırın. Hareket deneylerini otomatik test yerine koymayın; ayrı yetkilendirme, sınırlandırılmış plan, gözlem ve acil durdurma gereklidir.

Yeni deney için tarih, dal/commit, kullanılan harita, `enable_motion` durumu, başlangıç/hedef, sensör tazeliği, sonuç, başarısızlık nedeni ve ham verinin yerel konumunu yazın. Kısa, kanıta dayalı Markdown raporu `reports/` altında tutun; büyük JSON/ekran görüntüsü/log dosyaları yerelde kalır ve gerekiyorsa ekipçe ayrı paylaşılır. Simülasyon hatasında yeniden üretme komutu, çalışma zamanı düğümleri, ilgili zaman damgaları, beklenen/gözlenen davranış ve güvenlik durumu kaydedin. Gizli bilgi veya kişisel masaüstü görüntüsü eklemeyin.

İlk commit ve uzak depo ayarları için [GitHub kontrol listesi](GITHUB_CHECKLIST.md) kullanılmalı. Şu anda hiçbir remote, commit veya push bu hazırlık çalışmasının parçası değildir.
