# Geliştirme yol haritası

Tüm aşamalar **beklemede**. İşler için ayrı dal ve incelemeli PR önerilir. Planlama sonucu sürüş başarısı sayılmaz.

| Aşama / durum | Amaç | Gerekli işler | Bağımlılıklar | Kabul ölçütü |
| --- | --- | --- | --- | --- |
| 5I / Beklemede | B geçidinde kontrollü fiziksel sürüş | Yerel maliyet haritasını geçitte ölç, taze küp/AMCL/TF durumunu doğrula, denetleyiciyle uyumlu kısa aşamalar ve bağımsız ayak izi kontrolü hazırla, acil durdurma ile sınırlandırılmış deneme yap | 5H B planlama adayı; güvenlik sistemi | Her iki yönde izlenen kısa sürüş, yeterli ölçülmüş açıklık, küp hareketi ve beklenmedik temas yok, güvenli duruş |
| 5J / Beklemede | A geçidindeki açıklığı çöz | Yaklaşım rotasını ve yön belirsizliğini ölç, güvenli hizalama/planlama yöntemi geliştir, yerel harita ve denetleyiciyi doğrula | 5I ölçüm yöntemi, taze lokalizasyon | Başlangıçtan A'ya ve iki yönlü A geçişine tüm güvenlik marjlarıyla bağımsız onay; ardından ayrı sınırlandırılmış sürüş |
| 6A / Beklemede | Kamerayla küpe güvenli yaklaşım | Renk/kimlik/TF/tazelik kontrolü, yaklaşım hedefi, tam ayak izi ve küp mesafesi doğrulaması | 5I–5J güvenli rota; 5G izleme | Taze gözlemli, çarpışmasız yaklaşımın ölçülmüş tekrarlı denemeleri |
| 6B / Beklemede | İlk küpü eşleşen hedefe kontrollü it | Hizalama, temas, yörünge, durdurma, bağımsız Gazebo hedef ölçümü | 6A | Tek küpün hedefe otonom teslimi ve güvenli duruşu tekrarlanabilir |
| 6C / Beklemede | Üç küpün yönetimi ve teslimi | Görev sırası, renk hedef eşlemesi, yeniden planlama ve hata toparlama | 6B | Üç küpün doğru hedeflere otonom teslimi |
| 7 / Beklemede | Tam yarışma koşusu | Yeniden başlatma, dayanıklılık, süre ve hata kaydı | 6C | Sekiz dakika sınırı içinde tam otonom koşu bağımsız ölçümle doğrulanır |
| Son teslim / Beklemede | Teknik rapor, sunum ve video | Mimari, deney, sınırlama ve gösterim belgelerini derle | Önceki aşamalardan gerçek sonuçlar | Kanıtlarla tutarlı rapor ve gösterim |

Görev dağıtımı: navigasyon ekibi 5I–5J, algı ekibi 6A, görev/itme ekibi 6B–6C üzerinde çalışabilir; ortak harita, sahne ve robot geometrisi değişiklikleri önceden incelenmelidir.
