# Projekt Dyplomowy
Repozytorium zawiera wszystkie pliki wykorzystane w trakcie realizacji projektu dyplomowego.

## Arduino
Folder 'arduino' zawiera skrypt wykorzystany w ramach układu przełączającego.

## Jetson Nano

W folderze 'jetson' zostały umieszczone pliki wykorzystywane na platformie Jetson Nano. Skrypt kontrolera wykorzystywanego w ramach symulacji HIL zrealizowany został w ramach 'node' ROS2 znajdującego się w folderze ros2_ws. Pozostałe skrypty znajdujące się w folderze odpowiadają za wyświetlanie przetworzonego strumienia wideo oraz jego zapis. Znajduje się tam również zmodyfikowany skrypt kontrolera przystosowany do współpracy z rzeczywistym sprzętem.

## Środowisko symulacyjne

Wszystkie pliki wymagane do uruchomienia środowiska symacyjnego wykorzystanego w ramach projektu znajdują się w folderze 'ros2_px4_inz'. W celu zbudowania kontenera należy wykonać kroki opisane w [oryginalnym repozytorium](https://github.com/vision-agh/ros2_px4) po czym wywołać komendę
```bash
. post_build.sh
```

Aby uruchomić wybrany scenariusz testowy należy wykorzystać jeden z poniższych skryptów znajdujących się w folderze ros2_ws.
```bash
. launch_static.sh
. launch_circle.sh
. launch_eight.sh
```
Po uruchomieniu Gazebo należy wywołać poniższe komendy w celu uzbrojenia drona.
```bash
commander mode manual
commander arm -f
```
