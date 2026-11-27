# Deneme araclari (26-27 Kasim)

Bu klasordeki betikler test amaclidir; paketin parcasi degildir (colcon bunlari derlemez).
Hepsi saha koordinati kullanir; harita = saha - baslangic (-2.40, -1.40).
Gazebo'nun gercek konumu yalnizca sonucu olcmek icin okunur, kontrolde kullanilmaz.

| Dosya | Ne yapar |
| --- | --- |
| `dene.sh` | Eski surecleri temizler, Nav2 + Gazebo + RViz'i acar, hazir olmasini bekler, gozetmeni kapatip verilen betigi calistirir |
| `gecit_testi.py` | 5I + 5J: A ve B gecitlerinden iki yonlu Nav2 gecisi (7 hedef) |
| `tam_saha.py` | 5K: uc kup yaklasma pozu + uc hedef bolge onu + gecitler (10 hedef) |
| `teslim.py` | 6A-6F: kupleri Nav2 + kamera + kontrollu itme ile teslim eder, sure olcer |

Kullanim (Ubuntu 22.04 / Humble, betikleri ev klasorune kopyaladiktan sonra):

    ~/dene.sh ~/gecit_testi.py
    ~/dene.sh ~/tam_saha.py
    ~/dene.sh ~/teslim.py yesil kirmizi mavi

WSL'de her denemeyi temiz baslatmak icin PowerShell'den:

    wsl --shutdown; wsl -d Ubuntu-22.04 -- bash -ic "~/dene.sh ~/teslim.py yesil kirmizi mavi"

WSL'de Gazebo Fortress ekran karti ile cokuyor (ogre2 GL3Plus copyTo); `~/.bashrc`'ye
`export LIBGL_ALWAYS_SOFTWARE=1` eklenmeli. Bu durumda simulasyon gercek zamanin %35-60'i hizinda calisir.

ONEMLI: `dene.sh` testte `nav2_safety_supervisor`'i kapatir ve `/nav2_cmd_vel`'i dogrudan `/cmd_vel`'e
aktarir. Bu, resmi sistem icin degil, yalnizca deneme icindir (ekip karari bekliyor).
