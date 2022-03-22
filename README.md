# sim_physx
Simulator on Unity + PhysX communicating with ROS

## 概説
- 本シミュレータは自律施工技術開発基盤OPERA（Open Platform for Earth work with Robotics and Autonomy）の一部であり、どなたでも利用可能です
- シミュレータプラットフォームに[Unity](https://unity.com/)、物理エンジンに[Nvidia PhysX](https://www.nvidia.com/ja-jp/drivers/physx/physx-9-19-0218-driver/)を利用しています
- Unityを利用するため、利用者が所属する組織に応じたUnityのライセンスが必要です。詳細は[Unityの公式サイト](https://store.unity.com/ja)をご確認の上、利用登録をしてください。
- デモ動画を入れる

## インストール方法
### 1. Unity(ver:2020.3.16f1)のインストール
使用しているPCのOSに応じて以下の通りUnityHubをインストールする


- windows 又は Macの場合: [https://unity3d.com/jp/get-unity/download](https://unity3d.com/jp/get-unity/download)
- Linuxの場合(Linux版は動作確認していない):[https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download)
 
 上記のサイトにて"Download Unity Hub"をクリックし、最新の`UnityHub.AppImage`をダウンロード後、実行権限を付与する
  Then add execution permission for `UnityHub.AppImage` by following command.
  ```bash
  $ sudo chmod +x UnityHub.AppImage
  ```
  UnityHub.AppImageを実行する
   ```bash
   $ ./UnityHub.AppImage
   ```
  UnityHubを利用するためのライセンス認証手続きを行った後、Unity Editor（version: `2020.3.16f1`）を以下のアーカイブサイトより選択してインストールする

[https://unity3d.com/get-unity/download/archive](https://unity3d.com/get-unity/download/archive)

- Unity Editorのインストール方法も要るか？ 

### 2. Projectファイルの開き方

### 3. Sceneファイルの選択

### 4. ROS-TCP-Connectorの設定
- メニューからRobotics > ROS Settingを開き"ROS IP Address", "ROS Port"のところにROS側のIPアドレスおよびポート番号(defaultは10000)を入力する
- ROS-TCP-Connectorのアイコン部分の画像を入れる

![ros_ip_setting](https://user-images.githubusercontent.com/24404939/159395478-46617a2f-b05c-4227-9fc9-d93712dc4b9f.jpg)

### 5. ROSとの連携方法
- 【初回のみ】ROS側で[ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)パッケージをcloneし、buildとセットアップを行う。
  ```bash
  $ git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
  $ cd ./ROS-TCP-Endpoint/
  $ sudo chmod +x setup.py
  $ ./setup.py
  $ catkin build ros_tcp_endpoint
  $ source ../../devel/setup.bash
  ```
- ROS側でendpoint.launchを実行する
  ```bash
  $ roslaunch ros_tcp_endpoint endpoint.launch
  ```
- Unity Editor上部の実行ボタンをクリックする

![play_icon](https://user-images.githubusercontent.com/24404939/159396113-993ff0b2-d2bb-4567-ac68-0eafc9f524ac.png)
- ROS側で、対応する建機のunity用launch ファイルを起動する
  - 油圧ショベル
  ```bash
  $ roslaunch zx120_unity zx120_standby.launch
  ```
  - クローラダンプ
  ```bash
  $ roslaunch ic120_unity ic120_standby.launch
  ```
  - 油圧ショベルとクローラダンプの両方
  ```bash
  $ roslaunch zx120_ic120_standby.launch
  ```
