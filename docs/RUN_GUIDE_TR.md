# Çalıştırma kılavuzu

Her terminalde `source /opt/ros/humble/setup.bash` ve derleme sonrasında `source install/setup.bash` çalıştırın. Komutlar çalışma alanı kökünden verilmiştir. Aynı anda yalnızca **bir** Gazebo/Nav2 başlatması çalıştırın; Nav2 başlatması Gazebo'yu da açar.

```bash
cd ~/robotics_competition_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Modlar

| Mod | Başlatma | Kullanım |
| --- | --- | --- |
| Temel simülasyon | `ros2 launch competition_bringup competition.launch.py gui:=false rviz:=true` | Robot, sensörler, köprüler; SLAM/Nav2 yok |
| Çevrimiçi haritalama | `ros2 launch competition_bringup competition_mapping.launch.py gui:=false rviz:=true` | Gazebo + SLAM Toolbox; özgün kayıtlı haritayı yeniden üretmez |
| Kayıtlı harita ile navigasyon | Aşağıdaki komut | Gazebo + Map Server + AMCL + Nav2; hareket kapalı |
| Yalnızca planlama | Navigasyon modunda `enable_motion:=false`, yalnızca `ComputePathToPose` | Yol üretimi ve bağımsız ayak izi kontrolü; sürüş yok |
| Hareketli deneme | `enable_motion:=true` ayrı aşama | Önce taze sensör/AMCL/küp, maliyet haritaları, ayak izi, acil durdurma ve sınırlandırılmış deneme doğrulanmalı |

Özgün makinede kullanılan kayıtlı harita Nav2 komutu:

```bash
ros2 launch competition_bringup competition_nav2.launch.py gui:=false rviz:=true enable_motion:=false map:=$PWD/maps/competition_mapping/20261125T103259Z_572d1928/map.yaml
```

Bu varsayılan harita seçiminde başlatıcı çalışan `phase5b_nav2_map.yaml` metadata dosyasını kullanır; PGM yolu özgün makineye mutlak bağlıdır. Yeni bilgisayarda [SETUP_TR](SETUP_TR.md) içindeki taşınabilir örneği `map:=` ile seçin ve harita hücrelerini doğrulayın. `rviz:=true` Nav2 görünümünü açar. Temel/haritalama başlatmalarında da `rviz:=true` kullanılabilir; ayrı RViz işlemi gerekiyorsa `rviz:=false` ile açıp `rviz2 -d install/competition_bringup/share/competition_bringup/rviz/competition_nav2_clear.rviz` çalıştırın.

## Çalışma zamanı kontrolleri

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 action list
```

AMCL'nin durağan robotta son önbellek pozunu tekrar vermesi yeni ölçüm anlamına gelmez; mesaj zamanını ve TF'yi kontrol edin. Planlama için `/compute_path_to_pose` eylemi vardır. Canlı planlama aracı `src/competition_bringup/tools/phase5g_plan_only_recovery.py --goal-x <x> --goal-y <y> --output /tmp/plan.json` yalnızca hareket kapalıyken ve uygun oturum hazırken kullanılır; belge örneğindeki hedefler gerçek güvenli hedef iddiası değildir. 5H'nin geçit deneyi için `phase5h_passage_probe.py` kaynak kodu ve [5H raporu](../reports/phase5h_narrow_passage_validation.md) incelenmeli; bu araç hipotetik başlangıçlar kullanır ve rota sürdürmez. `NavigateToPose` göndermeyin.

```bash
python3 -m pytest -q src/competition_bringup/test
```

Kapatma: başlatmanın çalıştığı terminalde Ctrl-C; süreçlerin gerçekten bittiğini `ros2 node list` ve `pgrep -af 'ign gazebo|planner_server|controller_server'` ile kontrol edin. Sonraki başlatmadan önce eski Gazebo/Nav2 süreçlerinin kapanmasını bekleyin. Acil durdurma, hız koruyucusu ve hareket güvenliği katmanlarını devre dışı bırakmayın.
