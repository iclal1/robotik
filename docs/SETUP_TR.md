# Kurulum (Ubuntu 22.04)

Desteklenen geliştirme ortamı Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress ve Python 3'tür. Bu belge paket kurmaz; aşağıdaki komutlar yeni bir makinede kullanıcı tarafından çalıştırılır. `ign gazebo` Fortress komutu gerekir; Garden `gz sim` bu çalışma alanının başlatma komutu değildir.

## Bağımlılıklar

`src/competition_bringup/package.xml` şu çalışma bağımlılıklarını bildirir: `competition_robot_description`, `competition_simulation`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `slam_toolbox`, `nav2_map_server`, `nav2_amcl`, `nav2_planner`, `nav2_controller`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `nav2_regulated_pure_pursuit_controller`, `nav2_msgs`, `action_msgs`, `python3-opencv`, `python3-numpy`, `xacro`, `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `vision_msgs`, `visualization_msgs`, `rviz2`. Üç paket `ament_cmake` kullanır. Kaynaktaki testler ayrıca `pytest` kullanır; `colcon` ve `rosdep` geliştirme araçlarıdır. Paket manifestleri belirli apt sürümlerini sabitlemez.

```bash
source /opt/ros/humble/setup.bash
rosdep update
cd ~/robotics_competition_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | rg 'competition_|nav2_|slam_toolbox|ros_gz'
ign gazebo --version
python3 -c 'import cv2, numpy, pytest; print(cv2.__version__, numpy.__version__, pytest.__version__)'
```

`rosdep` sistem paketini bulamazsa önce doğru Humble ve Fortress depolarının etkin olduğunu, `/opt/ros/humble/setup.bash` dosyasının varlığını ve `rosdep check --from-paths src --ignore-src` çıktısını kontrol edin. Dağıtım/apt paket eşlemelerini tahmin ederek rastgele sürüm kurmayın. RViz ekranı açılmazsa masaüstü oturumunu ve OpenGL sürücüsünü kontrol edin; başsız doğrulama için `rviz:=false` kullanın.

## Çalışma alanı ve taşınabilir harita

`src/` üç ROS paketidir; `maps/` kaydedilmiş haritaları, `reports/` deney raporlarını, `docs/` kılavuzları, `scripts/` erken doğrulama araçlarını içerir. `build/`, `install/`, `log/` yerel üretimdir.

Özgün harita YAML'si kendi PGM'sine göreli `image: map.pgm` içerir ve taşınabilir; ancak Humble Map Server için bilinmeyen 275 hücreyi koruyan `src/competition_bringup/config/phase5b_nav2_map.yaml` mutlak `/home/ik/...` PGM yolunu kullanır. Varsayılan Nav2 başlatması bu metadata dosyasını seçer. Başka bilgisayarda özgün haritaya dokunmadan yanındaki [taşınabilir örneği](../maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml) kullanın:

```bash
cd ~/robotics_competition_ws
ros2 launch competition_bringup competition_nav2.launch.py gui:=false rviz:=true enable_motion:=false map:=$PWD/maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml
```

Örnek `image: map.pgm`, `free_thresh: 0.19` ve özgün metadata ile aynı çözünürlük/orijini kullanır. Çalışma anında haritanın 119×79 ve 275 bilinmeyen hücreyle açıldığını ayrıca doğrulayın. Farklı harita parametresi seçildiğinde başlatıcının varsayılan haritaya özel SHA denetimi çalışmaz; özgün dosyaların hash'ini [GitHub kontrol listesi](GITHUB_CHECKLIST.md) üzerinden ayrıca doğrulayın. `map.yaml`, `map.pgm` veya çalışan Nav2 yapılandırmasını değiştirmeyin.

Beklenen başlatma: Gazebo sahası ve ROS köprüleri, kamera/LiDAR, sonra Map Server, AMCL, planlayıcı ve güvenlik düğümleri açılır; `enable_motion:=false` ile sürüş komutları geçmez. [Çalıştırma kılavuzu](RUN_GUIDE_TR.md) modları ayırır.
