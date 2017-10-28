
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>
#include <sstream>

#include "rpi_v4l2_cam.h"

/* Static変数 */
ros::Publisher cam_pub;
sensor_msgs::CompressedImage image;
unsigned long count = 0;

/* define */
#define DEFAULT_WIDTH   (640)
#define DEFAULT_HEIGHT  (480)

/**
 * @brief           カメラコールバック関数
 * @param[in]       *pic_data               ST_PICTURE_INFOアドレス
 * @author          2017/10/09              新規作成
 * @note            カメラデバイスから画像を取得できた時に呼び出される
 */
void camera_callback( void* pic_data )
{
    ST_PICTURE_INFO* pic = reinterpret_cast<ST_PICTURE_INFO*>( pic_data );// データの受け取り
    image.header.stamp = ros::Time::now();              // ROSメッセージに現在時刻を設定
    image.header.frame_id = "picam";                    // フレームIDは決め打ちで"picam"
    image.header.seq = count++;                         // 配信回数を加算
    image.format = "jpeg";                              // 圧縮形式は決め打ちで"jpeg"
    unsigned char* work = reinterpret_cast<unsigned char*>( pic->addr );
    image.data.assign( &work[0], &work[ pic->length ] );// 配信データに画像データを格納
    cam_pub.publish(image);                             // Publish！！
}

/**
 * @brief           メイン関数
 * @param[in]       argc,argv               実行時引数
 * @author          2017/10/09              新規作成
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "compressed_picam");/* ノードの名前 */
    ros::NodeHandle n;
    cam_pub = n.advertise<sensor_msgs::CompressedImage>("compressed_picam/compressed", 10);/* パブリッシャの設定 */

    std::string size_mode;
    int width, height;

    /* カメラ解像度の取得 */
    /* 自由に解像度を指定されるとエラーが出るので、設定可能な選択肢しか提供しない */
    n.getParam( "/compressed_picam/resolution", size_mode );// 撮影解像度がParameterで指定されていれば従う
    if( size_mode == "VGA" ){                              // VGA設定の場合
      width = 640;
      height = 480;
    }else if( size_mode == "SVGA"){                        // SVGA設定の場合
      width = 800;
      height = 600;
    }else if( size_mode == "XGA"){                         // XGA設定の場合
      width = 1024;
      height = 768;
    }else if( size_mode == "QVGA"){                        // QVGA設定の場合
      width = 1280;
      height = 960;
    }else{                                                 // Parameterに設定が無い場合は初期値
      width = DEFAULT_WIDTH;
      height = DEFAULT_HEIGHT;
    }

    ROS_INFO("Camera mode is %s (%dx%d)",size_mode.c_str(),width,height);// モードの表示

    rpi_v4l2_cam* camera = new rpi_v4l2_cam();             // V4L2カメラクラスの実体作成

    image.data.reserve( width * height );                  // とりあえず画像サイズ分のメモリ空間を確保しておく

    camera->init( width, height );                         // カメラの初期化
    camera->start_stream( camera_callback );               // カメラの撮影開始

    ros::Rate loop_rate(1);                                // 1Hzで起きて少し作業する

    int prev_count = 0;

    while( ros::ok() ){                                    // メインループ
      if( (prev_count + 100) < count  ){                   // 適当な間引き
        ROS_INFO("frame count=%d",count);                  // 動作している事を確認するためのログ（別に無くても良い）
        prev_count = count;
      }
      ros::spinOnce();                                     // 今のところ必要無いのだけど忘れそうなのでspin実装しておく
      loop_rate.sleep();                                   // 寝る
    }
    delete camera;                                         // カメラ解放処理

    return 0;
}
