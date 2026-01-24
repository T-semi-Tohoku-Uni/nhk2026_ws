# nhk2026_ws


# 環境構築
## 実行に必要なライブラリ
- ros2 humble
- rosdep
- gazebo

#
### ros2 humble
[公式の手順](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)にしたがってインストール
追加でこちらも実行
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```


### rosdep
```bash
sudo apt-get install python3-rosdep
```
初期化と更新もしておく
```bash
sudo rosdep init
rosdep update
```

### colconのインストール
```bash
sudo apt install -y python3-colcon-common-extensions
```

### BaheaivorTreeのインストール
```bash
sudo apt install libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev libnanoflann-dev
cd
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
git checkout tags/4.7.0
mkdir build
cd build
cmake ..
make -j8
sudo make install
cd nhk2026_ws
```



### gazeboのインストール
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### rosのworkspaceを作成
```bash
cd
mkdir -p nhk2026_ws/src
```
各パッケージは`nhk2026_ws/src/`においていきます



## 環境設定
## 実機・シミュレーション共通の設定
エラー，warningは色をつける
```bash
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> ~/.bashrc
```

### シミュレーションで実行する場合
gazeboに表示するモデルのパスを設定
```bash
echo 'export IGN_GAZEBO_RESOURCE_PATH=~/nhk2026_ws/src/nhk2026_sim/models/' >> ~/.bashrc
```
`WITH_SIM`環境変数を`1`に設定（`0`にすると実機バージョンでビルドされるので注意）
```bash
echo 'export WITH_SIM=1' >> ~/.bashrc
``` 
GPUがついていないパソコンで実行する場合は、次のコマンドを実行
```bash
echo 'LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
```



## 依存する外部ライブラリのインストール
```bash
rosdep install --from-paths src -y --ignore-src
```

## 実行方法
（初回のみ全てのパッケージをビルド）
```bash
cd
cd nhk2026_ws
colcon build --symlink-install
```


実行
```bash
source install/setup.bash
ros2 launch nhk2026_core cpu_sim.launch.py
```
