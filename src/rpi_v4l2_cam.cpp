#include    <iostream>
#include    <cstdio>
#include    <string>

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <sys/types.h>
#include    <sys/stat.h>
#include    <sys/ioctl.h>
#include    <sys/mman.h>
#include    <unistd.h>
#include    <fcntl.h>
#include    <errno.h>
#include    <pthread.h>
#include    <linux/videodev2.h>

#include    "rpi_v4l2_cam.h"


/**
 * @brief           コンストラクタ
 * @param[in]       device_name             デバイスファイル名
 * @author          2017/10/09              新規作成
 */
rpi_v4l2_cam::rpi_v4l2_cam( std::string device_name )
{
    is_initialized = false;
    is_capturing = false;
    pict_info = NULL;
    mmap_max = 0;

    dev_fd = open( device_name.c_str(), O_RDWR | O_NONBLOCK, 0 );

    if( dev_fd > 0 ){                                                          // デバイスオープンに成功した
        is_initialized = true;
    }else{                                                                     // デバイスオープンに失敗した
        printf("failed open device %s", device_name.c_str());
        exit( EXIT_FAILURE );
    }
}

/**
 * @brief           デストラクタ
 * @param[in,out]   なし
 * @author          2017/10/09 nomumu       新規作成
 */
rpi_v4l2_cam::~rpi_v4l2_cam()
{
    if( is_capturing ){
        stop_capture( dev_fd );
    }
    if( is_initialized ){
        close( dev_fd );
        is_initialized = false;
    }

    // デバイスから取得したマップを解放
    if( pict_info != NULL ){                                                   // メモリ取得済の場合
        for( int i=0 ; i<mmap_max ; ++i ){
            if( -1 == munmap( pict_info[i].addr, pict_info[i].length ) ){      // mmapの解放
                fprintf( stderr, "munmap %d, %s \n", errno, strerror(errno) );
                exit( EXIT_FAILURE );
            }
        }
        //バッファ管理メモリの解放
        delete[]  pict_info;                                                   // メモリの解放
    }
}

/**
 * @brief           カメラ初期化関数
 * @param[in]       width                   カメラに設定する画像幅
 * @param[in]       height                  カメラに設定する画像高
 * @param[in]       type                    カメラに設定する画像フォーマット
 * @author          2017/10/09 nomumu       新規作成
 */
bool rpi_v4l2_cam::init( unsigned int width, unsigned height, EN_PICTURE_TYPE type )
{
    struct v4l2_capability cap;
    struct v4l2_format fmt={0};
    ST_PICTURE_INFO* buffers = NULL;
    unsigned int check_size;
    bool result;

    if( !ioctl_cam( dev_fd, VIDIOC_QUERYCAP, &cap ) ){                         // カメラデバイス情報取得
        if( errno == EINVAL ){                                                 // カメラが無いエラー
            fprintf( stderr, "no V4L2 device\n" );
        }else{                                                                 // その他よくわからないエラー
            fprintf( stderr, "VIDIOC_QUERYCAP error %d, %s \n", errno, strerror(errno) );
        }
        return false;
    }

    if( !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ){                        // 画像が取れるカメラか（念押し）
        fprintf( stderr, "not video capture device\n" );
        return false;
    }

    /* カメラへ画像フォーマットを指定する */
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;                                    // 画像のキャプチャ
    fmt.fmt.pix.width = width;                                                 // 幅
    fmt.fmt.pix.height = height;                                               // 高さ

    switch( type ){
    case CAM_TYPE_RAW:                                                         // RAW指定の場合
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        break;
    case CAM_TYPE_JPEG:                                                        // JPEG指定の場合
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
        break;
    case CAM_TYPE_YUYV:
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;                           // YUV指定の場合
        break;
    default:                                                                   // 画像フォーマットは必ず指定して欲しい
        return false;
    }
    fmt.fmt.pix.field = V4L2_FIELD_ANY;                                        // 画像フィールドオーダ,今回はあまり気にしなくて良い

    if( !ioctl_cam( dev_fd, VIDIOC_S_FMT, &fmt ) ){                            // 画像フォーマットを設定する
        fprintf( stderr, "pixel format not supported error %d, %s \n", errno, strerror(errno) );
        return false;
    }

    /* 戻ってきたデータを確認する */
    check_size = fmt.fmt.pix.width;
    if( fmt.fmt.pix.bytesperline < check_size ){
        fmt.fmt.pix.bytesperline = check_size;
    }
    check_size = (fmt.fmt.pix.bytesperline * fmt.fmt.pix.height);
    if( fmt.fmt.pix.sizeimage < check_size ){
        fmt.fmt.pix.sizeimage = check_size;
    }

//  fprintf( stderr, "Video bytespreline = %d h=%d w=%d\n", fmt.fmt.pix.bytesperline,fmt.fmt.pix.height,fmt.fmt.pix.width );

    if( !init_mmap( dev_fd ) ){                                                // mmapの初期化
        return false;
    }

    return true;
}

/**
 * @brief           ioctl関数
 * @param[in]       fd                      ファイルディスクリプタ
 * @param[in]       request                 リクエストコード
 * @param[in]       arg                     リクエスト内容
 * @return          true                    成功
 * @return          false                   失敗
 * @author          2017/10/09 nomumu       新規作成
 */
bool rpi_v4l2_cam::ioctl_cam( int fd, int request, void* arg )
{
    int res;
    do{
        res = ioctl( fd, request, arg );                                       // リクエストしてみる
    }while( (res == -1) && (errno == EINTR) );                                 // 何か割り込まれたで...

    if( res == -1 ){
        return false;
    }else{
        return true;
    }
}

/**
 * @brief           共有メモリ初期化処理
 * @param[in]       fd                      ファイルディスクリプタ
 * @return          true                    成功
 * @return          false                   失敗
 * @author          2017/10/09              新規作成
 */
bool rpi_v4l2_cam::init_mmap( int fd )
{
    struct v4l2_requestbuffers req = {0};

    //デバイスにmmap用のキャプチャバッファをリクエストする
    req.count  = IMAGE_BUF_NUM;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if( !ioctl_cam( fd, VIDIOC_REQBUFS, &req ) ){                              // ioctlリクエスト
        if( EINVAL == errno ){
            fprintf( stderr, "not support memory mapping\n" );
        }else{
            fprintf( stderr, "VIDIOC_REQBUFS error %d, %s %d\n", errno, strerror(errno), fd );
        }
        return false;
    }

    if( req.count < 2 ){                                                       // 2面は欲しい       
        fprintf( stderr, "Insufficient buffer memory\n" );
        return false;
    }

    mmap_max = req.count;
    pict_info = new ST_PICTURE_INFO[ req.count ];                              // 取得する面数分のメモリ管理領域を取得
    if( !pict_info ){
        fprintf( stderr, "Out of memory\n" );
        return false;
    }

    for( int index=0 ; index < req.count ; ++index ){                          // デバイスのキャプチャバッファをマップする
        struct v4l2_buffer buf = {0};
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;                                // ビデオキャプチャバッファの要求
        buf.memory = V4L2_MEMORY_MMAP;                                         // mmapアドレスが欲しい
        buf.index = index;                                                     // インデックス指定

        if( !ioctl_cam( dev_fd, VIDIOC_QUERYBUF, &buf ) ){
            fprintf( stderr, "VIDIOC_QUERYBUF error %d, %s \n", errno, strerror(errno) );
            return false;
        }
        
        pict_info[ index ].length = buf.length;                                // 提供されたメモリ空間のサイズ
        pict_info[ index ].addr = mmap( NULL , buf.length, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_fd, buf.m.offset );// メモリ空間のアドレスを共有する

        if( pict_info[ index ].addr == MAP_FAILED ){
            fprintf( stderr, "mmap error %d, %s \n", errno, strerror(errno) );
            return false;
        }
    }
    return true;
}

/**
 * @brief           キャプチャ開始処理
 * @param[in]       fd                      ファイルディスクリプタ
 * @param[in]       max                     mmap取得数
 * @return          なし
 * @author          2017/10/09              新規作成
 */
bool rpi_v4l2_cam::start_capture( int fd, int max )
{
    unsigned int i;
    enum v4l2_buf_type type;
    struct v4l2_buffer buf = {0};

    for( i=0 ; i<max ; ++i ){
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;//バッファ番号指定

        if( !ioctl_cam( fd, VIDIOC_QBUF, &buf ) ){                             // ビデオバッファを画像撮影キューに接続する
            fprintf( stderr, "VIDIOC_QBUF %d, %s %d\n", errno, strerror(errno), fd );
            return false;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if( !ioctl_cam( fd, VIDIOC_STREAMON, &type ) ){                            // キャプチャ開始
        fprintf( stderr, "VIDIOC_STREAMON %d, %s \n", errno, strerror(errno) );
        return false;
    }
    is_capturing = true;

    return true;
}

/**
 * @brief           キャプチャ停止処理
 * @param[in]       fd                      ファイルディスクリプタ

 * @return          なし
 * @author          2017/10/09              新規作成
 */
void rpi_v4l2_cam::stop_capture( int fd )
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if( !ioctl_cam( fd, VIDIOC_STREAMOFF, &type ) ){                           // キャプチャ停止
        fprintf( stderr, "VIDIOC_STREAMOFF %d, %s \n", errno, strerror(errno) );
        /* Nothing todo... */
    }
    is_capturing = false;
}

/**
 * @brief           カメラストリーム開始処理
 * @param[in]       callback_func           コールバック先関数アドレス
 * @return          ストリーム管理情報
 * @author          2017/10/09              新規作成
 */
ST_STREAM_INFO* rpi_v4l2_cam::start_stream( void (*callback_func)(void* data) )
{
    pthread_attr_t thread_attr;

    if( callback_func != NULL ){                                               // 念のためNULLチェック
        /* スレッドに渡すための情報作成 */
        stream_info.fd = dev_fd;
        stream_info.queue_num = mmap_max;
        stream_info.callback_func = callback_func;
        stream_info.queue_data = pict_info;
        stream_info.pClass = this;
        stream_info.is_terminated = 0;

        (void) pthread_attr_init( &thread_attr );
        (void) pthread_attr_setstacksize( &thread_attr, THREAD_STACK_SIZE );

        if( pthread_create( &stream_info.thread_id,                            // 画像取得用のスレッドを作成する
                            &thread_attr,
                            stream_thread,
                            (void*)&stream_info ) != 0 ){
            //スレッド生成に失敗
            return NULL;
        }
    }

    return &stream_info;
}

/**
 * @brief           カメラストリーム終了処理
 * @param[in]       stream_info     ストリーム管理情報
 * @return          なし
 * @author          2014/10/03              新規作成
 */
void rpi_v4l2_cam::end_stream( ST_STREAM_INFO* stream_info )
{
    stream_info->is_terminated = 1;
    (void) pthread_join( stream_info->thread_id, NULL );
}


/**
 * @brief           ストリームデータを待つ処理
 * @param[in]       fd                      ファイルディスクリプタ
 * @retval          enWait_Signal           画像の更新あり(正常)
 * @retval          enWait_Timeout          画像取得タイムアウト(異常)
 * @retval          enWait_Error            画像取得異常(異常)
 * @retval          enWait_EINTR            画像待ちの間にシグナルを受信した(異常)
 * @author          2017/10/09              新規作成
 */
EN_WAIT_CAPTURE rpi_v4l2_cam::wait_capture( int waitfd )
{
    fd_set fds;
    struct timeval tv;
    int res_select;
    EN_WAIT_CAPTURE result = enWait_Error;

    FD_ZERO( &fds );
    FD_SET( waitfd, &fds);

    //タイムアウト設定(2秒)
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    res_select = select( (waitfd + 1), &fds, NULL, NULL, &tv );                // 画像データ待ち(ブロッキング)

    /* 画像の取得結果を判断する */
    if( FD_ISSET(dev_fd,&fds) ){
        if( (-1 == res_select) || (0 == res_select) ){ //"-1"：select異常,"0"：タイムアウト
            if( errno == EINTR ){
                result = enWait_EINTR;
            }else{
                result = enWait_Error;
            }
        }else{
            result = enWait_Signal;
        }
    }else{
        result = enWait_EINTR;
    }

    return result;
}


/**
 * @brief           カメラストリーム読み込みスレッド処理
 * @param[in]       p_arg                   スレッド引数アドレス(ストリーム管理情報アドレス)
 * @return          デバイスバッファ管理アドレス
 * @author          2017/10/09              新規作成
 */
static void* stream_thread( void* p_arg )
{
    ST_STREAM_INFO* info = static_cast<ST_STREAM_INFO*>(p_arg);
    class rpi_v4l2_cam* p_this = info->pClass;
    int fd;
    void (*callback_func)(void* data);
    struct v4l2_buffer v4l2_buf = {0};
    int wait_res;

    fd = info->fd;
    callback_func = info->callback_func;

    p_this->start_capture( fd, info->queue_num );                              // キャプチャ開始(初回のキュー接続含む)

    while( !info->is_terminated ){
        wait_res = p_this->wait_capture( fd );
        if( wait_res == enWait_Error ){                                        // 読み込み可能になるのを待つ
            //データ待ちが異常終了した時
            info->is_terminated = 1;                                           // スレッドを抜ける
            break;
        }else if( wait_res == enWait_EINTR ){                                  // 致命的ではない異常が発生した
            fprintf( stderr, "v4l2 EINTR(%d)\n", errno );                      // 異常の報告だけする
            continue;                                                          // 待ち処理に戻る
        }

        /* 画像がキャプチャされたので取り出す */
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        if( !p_this->ioctl_cam( fd, VIDIOC_DQBUF, &v4l2_buf ) ){               // カメラバッファをキューから外す
            /* デバイス操作に失敗 */
            switch (errno) {
            case EAGAIN:
            case EIO: //EIO ignored
            default:
                info->is_terminated = 1;                                       /* スレッドを抜ける                      */
                break;
            }
        }

        /* コールバック */
        info->queue_data[ v4l2_buf.index ].length = v4l2_buf.bytesused;        /* 圧縮率で毎回サイズが変わるので必ず格納する */
        (*callback_func)( (void*)&info->queue_data[ v4l2_buf.index ] );        /* コールバック処理 */
        
        /* 画像を取り出し終わったのでカメラバッファにキューをつなぐ */
        if( !p_this->ioctl_cam( fd, VIDIOC_QBUF, &v4l2_buf) ){
            /* デバイス操作に失敗 */
            info->is_terminated = 1;                                           /* スレッドを抜ける */
            break;
        }
    }

    p_this->stop_capture( fd );

    return NULL;
}
