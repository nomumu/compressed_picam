Raspberry Pi Zero W 用 圧縮イメージカメラノード
====

## Description
「ROSではじめるホビーロボット#06」で紹介したRaspberry Pi Zero W で動作する圧縮画像カメラノードのサンプルです。 
カメラモジュールが出力した JPEG画像を直接Publishする事でCPU負荷の低減を図っています。
その代わりに通常のカメラノードが出力するRawImageは諦めて出力していません。
性能の高いPC等で画像処理可能な形に加工出来る場合に使用します。
Raspberry Pi Zero Wでのみ動作確認を行っています。

## Requirement
コンパイルにはROS環境の他に、libv4l-devも必要なので合わせてインストールして下さい。   
$ sudo apt-get install libv4l-dev -y   

## Usage
下記のように実行して下さい。   
$ rosrun compressed_picam compressed_picam   

カメラ解像度は下記の操作で変更する事が出来ます。rosrunで起動する前に設定して下さい。   
$ rosparam set /rpi_v4l2_cam/resolution "XGA"   
["VGA","SVGA","XGA","QVGA"]が設定出来ます。   

正しく動作していればrostopicコマンドで下記のように見えるはずです。   
$ rostopic list   
/compressed_picam/compressed   
/rosout   
/rosout_agg   

ROS Masterを同じくする別のPC（デスクトップ表示がある環境）から下記のコマンドを実行すると配信している画像を確認することが出来ます。   
$ export ROS_MASTER_URI=http://＜RaspberrypiのIP＞:11311   
$ export ROS_IP=＜RaspberrypiのIP＞   
$ rosrun image_view image_view image:=/compressed_picam _image_transport:=compressed _autosize:=true   

通信速度を確認したい場合は下記のコマンドが便利です。   
$ rostopic hz /compressed_picam/compressed   
