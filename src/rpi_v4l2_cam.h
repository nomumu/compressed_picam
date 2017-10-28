#define     THREAD_STACK_SIZE (1920*1080*3)/* 画像取得スレッドのスタックサイズ（大き目にしておく） */
#define     IMAGE_BUF_NUM   (3)

/* 画像フォーマット */
typedef enum {
    CAM_TYPE_RAW,                                          // RAWフォーマット(RGBだけど)
    CAM_TYPE_JPEG,                                         // JPEGフォーマット
    CAM_TYPE_YUYV,                                         // YUVフォーマット
    CAM_TYPE_NUM
} EN_PICTURE_TYPE;

/* 画像キャプチャ待ち結果 */
typedef enum {
    enWait_Signal,
    enWait_Timeout,
    enWait_Error,
    enWait_EINTR,
    enWait_NUM
} EN_WAIT_CAPTURE;

/* 画像取得情報構造体 */
typedef struct {
    unsigned long   length;
    void*           addr;
} ST_PICTURE_INFO;

class rpi_v4l2_cam;// 構造体に定義するのでプロトタイプ宣言

/* キャプチャスレッドに渡す情報構造体 */
typedef struct {
	int                 fd;                                // ファイルディスクリプタ
	int                 queue_num;                         // mmapで取得出来た空間の数
	void                (*callback_func)(void* data);      // コールバック先関数アドレス
	int                 is_terminated;                     // スレッド終了ステータス(0:実行中,else:スレッド終了要求)
	pthread_t           thread_id;                         // スレッド解放用のスレッドID
    ST_PICTURE_INFO*    queue_data;                        // 画像取得情報のアドレス
    class rpi_v4l2_cam* pClass;                            // クラス実体のアドレス
} ST_STREAM_INFO;

/* カメラキャプチャクラス */
class rpi_v4l2_cam {
public:
    rpi_v4l2_cam( std::string device_name="/dev/video0" );
    ~rpi_v4l2_cam();
    bool init( unsigned int width, unsigned height, EN_PICTURE_TYPE type=CAM_TYPE_JPEG );
    bool start_capture( int fd, int max );
    void stop_capture( int fd );
    EN_WAIT_CAPTURE wait_capture( int waitfd );
    static void capture_thread();
    bool ioctl_cam( int fd, int request, void* arg );
    ST_STREAM_INFO* start_stream( void (*callback_func)(void* data) );
    void end_stream( ST_STREAM_INFO* stream_info );

private:
    bool             is_initialized;                       // 初期化済みフラグ
    bool             is_capturing;                         // キャプチャ実行中フラグ
    int              dev_fd;                               // ファイルディスクリプタ
    int              mmap_max;                             // mmapで取得出来た空間の数
    ST_PICTURE_INFO* pict_info;                            // 画像取得情報
    ST_STREAM_INFO   stream_info;                          // キャプチャスレッドに渡す情報

    bool init_mmap(int fd );
};

static void* stream_thread( void* p_arg );
