/* 191202 デジタルx3 アナログx2　センサに改編済み */
/* 191204 microSDログ機能実装済み(FAT32) */
/* 191206 3.2[m/s]以上出せるよう変更 */

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "r8c38a_lib.h"                 /* R8C/38A 制御ライブラリ       */
#include "types3_beep.h"                /* ブザー追加                   */
#include "printf_lib.h"
#include "microsd_lib.h"

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* 定数設定 */

// 変更禁止:
#define     TRC_MOTOR_CYCLE     20000   /* 左前,右前モータPWMの周期     */
/* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* 左後,右後,ｻｰﾎﾞﾓｰﾀPWMの周期   */
/* 50[ns] * 20000 = 1.00[ms]    */

#define     FREE        1   /* モータモード　フリー         */
#define     BRAKE       0   /* モータモード　ブレーキ       */

#define     HIGH        1   /* 5V入力           */
#define     LOW         0   /* 0V入力           */

#define     ON          1
#define     OFF         0

#define     RIGHT       p1_addr.bit.b6
#define     LEFT        p0_addr.bit.b0
#define 	CENTER 		p0_addr.bit.b3

#define A 3
#define B 4
#define C 5
#define D 6

#define VR  ( get_ad(14) )

// 変更OK:
/* dipsw:　4?6 */
#define  	KP  	15      /* 比例  */
#define  	KI  	0     	/* 積分  */
#define 	KD  	50      /* 微分  */

#define 	BEEP 				1
#define 	ENC  				1   	/* エンコーダが有効か 1:有効 0:無効	*/
#define     ENCHU_ENABLE        0       /* 円柱標的の有効化				*/

#define     RUNNING_TIME        31000L  /* 走行時間(ms)                 */

#define     AD_1DEG             2       /* 1度あたりのA/D値の増分       */

#define  	SERVO_PWM_MAX   	100    	/* サーボの最大PWM        */
#define		LANCER_PWM_MAX		0		/* 槍の最大PWM  		*/

#define     CENTER_LNC          512		/* 槍の中心のA/D値		*/         

#define  	ROLLING  			775 /* カーブ検知の閾値 */

/* 的の角度 */
volatile int     ENCHU_ANGLE_AD = 786;			 /* 円柱標的に突っ込む際のボリューム値 */

#define     UP4BIT /* dipsw2上4bit */ ( dipsw_get2() >> 4 )  /* 平行標的に当てるまでの距離: */
#define     MIN4BIT/* dipsw2下4bit */ ( dipsw_get2()&0x0f )  /* ハーフライン検出から通過までのパルス数(272.5*2.5=25cm) */

// サーボ用
// Arduino nanoとの接続:(出力)
#define     SERVO_ANGLE_PORT    p5_addr.bit.b3
#define     SERVO_ZERO_PORT     p5_addr.bit.b2
#define     SERVO_MODE_PORT     p5_addr.bit.b1

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char convertBCD( int data );
void led_out( unsigned char led );
void servoSet( int num );
void fullColor_out( unsigned char type );
void traceMain( void );
void motor_r( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
void servoControl( void );
void servoControl2( void );
void stm_go( int degree );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
int getServoAngle( void );
int getAnalogSensor( void );
int diff( int pwm );
int getLancerAngle( void );
void hyouteki_check( void );
void lancerPwmOut( int pwm );
void lancerControl( void );
long map( long x, long in_min, long in_max, long out_min, long out_max );
void newcrank( void );
void sp( int l , int r );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/

/* データフラッシュ関連 */

volatile const char      *C_DATE = __DATE__;     /* コンパイルした日付           */
volatile const char      *C_TIME = __TIME__;     /* コンパイルした時間           */

volatile int             pattern       = 0;      /* マイコンカー動作パターン     */
volatile int             pattern_settings = 0;
volatile unsigned long   cnt_run       = 0;      /* タイマ用                     */
volatile unsigned long   cnt1          = 0;      /* タイマ用                     */
volatile unsigned long   check_cross_cnt = 0;    /* タイマ用                     */
volatile unsigned long   check_sen_cnt = 0;      /* タイマ用                     */
volatile unsigned long   check_enc_cnt = 0;      /* タイマ用                     */
volatile int             anchi_cross = 0;        /* 1:前回に片方のデジタルセンサが反応 0:デジタルの反応なし */
volatile int             hitcount      = 0;      /* ハーフラインを読んだ回数     */
volatile int             hyouteki_flag = 0;      /* 標的が垂直か平行かを見分ける(平行標的:0 垂直標的:1)*/
volatile int             heikou  = 0;
volatile int             stair_flag  = 0;        /* 1:大きく曲がっている 0: 直線 */
volatile int	   	     kyori_flug = 0;             	/* 各ラインの通過検出宣言  	 スタート前の初期設定   */
volatile int	   		 kyoritime  = 0;             	/* 各ラインの通過時間宣言    スタート前の初期設定   */


/* エンコーダ関連     */
volatile int             iTimer10     = 0;       /* 10msカウント用               */
volatile int             iEncoder     = 0;       /* 10ms毎の最新値               */
volatile int             iEncoderMax  = 0;       /* 現在最大値                   */
volatile long            lEncoderLine = 0;       /* ライン検出時の積算値       */
volatile long            lEncoderTotal = 0;      /* 積算値保存用                 */
volatile unsigned int    uEncoderBuff = 0;       /* 計算用　割り込み内で使用     */

/* サーボ関連       */
volatile int             iSensorBefore;          /* 前回のセンサ値保存           */
volatile int             iServoPwm;              /* サーボPWM値                */
volatile int             iAngle0;                /* 中心時のA/D値保存            */

/* サーボ関連2          */
volatile int             iSetAngle;
volatile int             iSetAngleAD;
volatile int             iAngleBefore2;
volatile int             iServoPwm2;

/* 槍(DCモータとボリュームad7) */
volatile int             iLancer0;				 /* 中心時のA/D値保存 			 */
volatile int			 iSetLancer;			 /* 目標のgetLancerAngle()の値   */
volatile int 			 iSetLancerAD;			 /* 目標のAD値					 */
volatile int			 iLancerPwm;			 
volatile int			 iLancerBefore;

/* TRCレジスタのバッファ */
volatile unsigned int    trcgrb_buff;            /* TRCGRBのバッファ             */
volatile unsigned int    trcgrd_buff;            /* TRCGRDのバッファ             */
volatile unsigned int    trcgrc_buff;

/* microSD関連変数 */
volatile int			msdFlag;				 /* 1:データ記録 0:記録しない */
volatile int			msdError;				 /* エラー番号記録 */


/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
volatile unsigned char   types_led;              /* LED値設定                    */
volatile unsigned char   types_dipsw;            /* ディップスイッチ値保存       */

/* 内輪差値計算用　各マイコンカーに合わせて再計算して下さい */
volatile const int revolution_difference[] = {   /* 角度から内輪、外輪回転差計算 */
  100, 98, 97, 95, 94,
  93, 91, 90, 88, 87,
  86, 84, 83, 82, 80,
  79, 78, 76, 75, 74,
  72, 71, 70, 68, 67,
  66, 65, 63, 62, 61,
  59, 58, 57, 55, 54,
  53, 51, 50, 49, 47,
  46, 45, 43, 42, 40,
  39
};

/* その他 */
volatile long    integral = 0;


/* フルカラーLED用(p6) */
volatile const int fullColor_data[] = {
  //  0:OFF 1:赤  2:緑　3:青  4:黄  5:紫  6:水色 7:白
  0x00, 0x01, 0x02, 0x04, 0x03, 0x05, 0x06,  0x07
};


/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
  int i, ret;
  char fileName[ 8+1+3+1 ]; /* 名前+'.'+拡張子+'\0' */
  unsigned char b;
  
  /* データフラッシュ処理用 */
  int r;
  unsigned char c;
  /**************************/
	
  /* マイコン機能の初期化 */
  init();                             /* 初期化                       */
  fullColor_out(0);
  setMicroSDLedPort( &p6, &pd6, 0 );  /* microSDモニタLED設定 */
  _asm(" FSET I ");                   /* 全体の割り込み許可           */
  initBeepS();                        /* ブザー関連処理               */
  init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
  
  /* microSD初期化 */
  ret = initMicroSD();
  if( ret != 0x00 ) msdError = 1;
  
  /* FAT32でマウント */
  if( msdError == 0 ){
	  ret = mountMicroSD_FAT32();
	  if( ret != 0x00 ) msdError = 2;	  
  }
  
  if( msdError != 0 ){
	/* microSD処理にエラーがあれば3秒間、LEDの点灯方法を変える */	  
  	cnt1 = 0;
	while( cnt1 < 3000 ){
		if( cnt1 % 200 < 100 ){
			led_out(0x03);
		}else{
			led_out(0x00);	
		}
			
	}
  }
  
  /* マイコンカーの状態初期化 */
  motor_mode_f( BRAKE, BRAKE );
  motor_mode_r( BRAKE, BRAKE );
  motor2_f( 0, 0 );
  motor2_r( 0, 0 );
  servoPwmOut( 0 );
  fullColor_out( 0 );
  setBeepPatternS( 0x8000 );
  timer_ms(10);
  iSetLancer = CENTER_LNC;
  iLancer0 = getLancerAngle();
  
  
  servoSet( 0 ); 

		
  ///////////////////////////////
  
  fullColor_out( 0 );
  while ( 1 ) {
	if( pattern >= 11 && pattern < 100 ){
		if( cnt_run >= RUNNING_TIME ){
			pattern = 101;	
			cnt1 = 0;
		}	
	}
	if( pattern >= 1 && pattern < 102){
		servoControl();
		servoControl2();
		servoPwmOut(iServoPwm);
	}
	  
    switch ( pattern ) {
      case 0:
        /* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if ( pushsw_get() ) {
			if( msdError == 0 ){
				/* microSDの空き領域から読み込み */
				i = readMicroSDNumber();
				if( i == -1 ){
					msdError = 3;	
				}	
			}
			if( msdError == 0 ){
				/* microSDの空き領域へ書き込み */
				i++;
				if( i >= 10000 ) i = 1;
				ret = writeMicroSDNumber(i);
				if(ret == -1 ){
					msdError = 4;	
				} else {
					/* ファイル名変換 */
					sprintf( fileName, "log_%04d.csv", i );	
				}
			}
			if( msdError == 0 ){
				/* ファイルのタイムスタンプセット */
				setDateStamp( getCompileYear( C_DATE ), getCompileMonth( C_DATE ),   getCompileDay( C_DATE ) );
				setTimeStamp( getCompileHour( C_TIME ), getCompilerMinute( C_TIME ), getCompilerSecond( C_TIME ) );	
				
				/* 書き込みファイル名作成 */
				// 書き込みしたい時間[ms]: x = 10[ms] : 64バイト
				// 60000msなら x = 60000 * 64 / 10 = 384000
				// 結果は512の倍数になるよう繰り上げする
				ret = writeFile( fileName, 384000 );
				if( ret != 0x00 ) msdError = 11;
				
				// microSD書き込み
				msdPrintf("[Natadekoko-ep3] Log Data\n");
				while( checkMsdPrintf() );  // msdPrintf完了待ち
				msdPrintf("Compile Date:");
				while( checkMsdPrintf() );  // msdPrintf完了待ち
				msdPrintf( C_DATE );
				while( checkMsdPrintf() );  // msdPrintf完了待ち
				msdPrintf("Time:");
				while( checkMsdPrintf() );  // msdPrintf完了待ち
				msdPrintf( C_TIME );
				while( checkMsdPrintf() );  // msdPrintf完了待ち
				msdPrintf("\n\nLineNo,Pattern,Left,Right,Cross,Analog,Angle,Encoder,v[m/s],kyoritime\n");
				while( checkMsdPrintf() );  // msdPrintf完了待ち
			}
          	setBeepPatternS( 0xcc00 );
          	cnt1 = 0;
          	pattern = 1;
          	break;
        }
		printf("left= %4d, an_l=%4d, center= %4d,an_r= %4d, right= %4d\r", check_leftline(), get_ad(13), center_inp(), get_ad(12), check_rightline());
//		printf("an_l=%4d, center= %4d,an_r= %4d,    sensor= %4d\r",  get_ad(13), center_inp(), get_ad(12), sensor_inp());

        
        break;

      case 1:
        /* スタンバイ:スタート3秒前 */

        led_out( 0x11 );
        setBeepPatternS( 0x8000 );  // 3
        timer_ms( 1000 );

        led_out( 0x33 );
        setBeepPatternS( 0x8000 );  // 2
        timer_ms( 1000 );

        led_out( 0x77 );
        setBeepPatternS( 0x8000 );  // 1
        timer_ms( 1000 );

        led_out( 0xff );
        setBeepPatternS( 0xffff );  // GO!
        timer_ms( 1000 );

        led_out( 0x00 );
		
		fullColor_out( 0 );
		
        check_sen_cnt = 0;
        check_enc_cnt = 0;
        lEncoderTotal = 0;
        lEncoderLine  = 0;
		hyouteki_flag = 0;
		if( msdError == 0 ) msdFlag = 1; /* データ記録開始 */
		iAngle0 = getServoAngle();  	/* 0度の位置記憶          */
		iLancer0 = getLancerAngle();  	/* 0度の位置記憶          */
		cnt_run = 0;
        cnt1 = 0;		
		pattern = 11;
        break;

      case 11:
        /* 通常トレース */
		servoPwmOut(iServoPwm);
        traceMain();
		fullColor_out( 0 );
		led_out( 0x00 );
        servoSet( 0 );
		if( check_crossline() ){
			heikou = 0;
			pattern = 20;
			cnt1 = 0;
			break;	
		}
        if ( check_rightline() && hyouteki_flag == 0 ) { /* 右ハーフラインチェック:平行標的*/
          pattern = 70;
		  lEncoderLine = lEncoderTotal;
          cnt1 = 0;
          servoSet( 1 );  // 平行振るだけ
          break;
        }
	
        if ( check_rightline() && hyouteki_flag == 1 ) { /* 右ハーフラインチェック:垂直標的*/
          pattern = 30;
		  lEncoderLine = lEncoderTotal;
          cnt1 = 0;
          servoSet( A );  // 垂直A
          break;
        }
		
        break;
	
	case 20:
	  	/* カーブ進入時 */
		setBeepPatternS( 0x8800 );
		servoPwmOut(iServoPwm);
		traceMain();
#if ENCHU_ENABLE
		
		if( cnt_run > ( RUNNING_TIME - 6000 ) ){  // 終了5秒前なら円柱標的へ
			pattern = 200;
			cnt1 = 0;
#if ENC
        	lEncoderLine = lEncoderTotal;
#endif // ENC
			break;	
		}
#endif  // ENCHU_ENABLE 
		
		kyori_flug = 0;
		led_out( 0xff );
		lEncoderLine = lEncoderTotal;
		hyouteki_flag = 1 - hyouteki_flag; // モード切替
		cnt1 = 0;
		pattern = 21;		
	  	break;
	
	case 21:
		servoPwmOut( iServoPwm );
		if( iEncoder >= 22 ){
			motor_mode_f(BRAKE,BRAKE);
			motor_mode_r(BRAKE,BRAKE);
			motor2_f(-50,-50);
			motor2_r(-70,-70);	
		}
		else{
			motor_mode_f(BRAKE,BRAKE);
			motor_mode_r(BRAKE,BRAKE);
			motor_f(diff(30),30);
			motor_r(diff(30),30);		
		}
		if( kyori_flug == 0 && (check_rightline()==1 || check_leftline()==1)  ){
            kyori_flug = 1;
        } else if( kyori_flug == 1 && (check_rightline()==1 || check_leftline()==1)){
            kyori_flug = 2;
		  /* クロスライン間の通過時間計測[ms] 白線手前から白線終わりまで */
            kyoritime = cnt1;
		}
        if( kyori_flug == 2 ){
            kyori_flug = 0;
			lEncoderLine = lEncoderTotal;
            cnt1 = 0;
            pattern = 22;
            break;
        }
		break;
	
	case 22:
		servoPwmOut( iServoPwm );
		setBeepPatternS( 0xcc00 );
        kyori_flug = 0;
        newcrank( );
        if( lEncoderTotal - lEncoderLine >= (273L * 2) ) {
            pattern = 23;  
			lEncoderLine = lEncoderTotal;
            break;
        }
		break;
	
	case 23:
		servoPwmOut( iServoPwm );
        kyori_flug = 0;
        newcrank( );
        if( lEncoderTotal - lEncoderLine >= (273L * 2) ) {
            pattern = 24;
			lEncoderLine = lEncoderTotal;
			
        }
		break;
	
	case 24:	
		servoPwmOut( iServoPwm );
        newcrank( );
		servoSet( 0 );
		if( check_rightline() || check_leftline() ){
			lEncoderLine = lEncoderTotal;
			cnt1 = 0;
			pattern = 25;	
		}
		break;
	
	case 25:
		servoPwmOut( iServoPwm );
        traceMain();
		servoSet( 0 );
		if( lEncoderTotal - lEncoderLine >= (273L * 2 )){
			cnt1 = 0;
			pattern = 11;	
			servoSet( 0 );
		}
		break;
	
	case 30:
		/* A標的 */
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( A ); // A標的
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 31;
		break;
	
	case 31:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_leftline() ){
			pattern = 40;
			cnt1 = 0;
			break;
		}
		servoSet( A ); // A標的
		if( lEncoderTotal - lEncoderLine >= (273L * 2) ){
			lEncoderLine = lEncoderTotal;
			pattern = 32;
			cnt1 = 0;
		}
		break;
	
	case 32:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_leftline() ){
			pattern = 40;
			cnt1 = 0;
		}
		break;
	
	case 40:
		/* B標的 */
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( B ); // B標的
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 41;
		break;
	
	case 41:
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( B ); // B標的
		if( check_rightline() ){
			pattern = 50;
			cnt1 = 0;
			break;
		}
		if( lEncoderTotal - lEncoderLine >= (273L * 2) ){
			lEncoderLine = lEncoderTotal;
			pattern = 42;
			cnt1 = 0;
		}
		break;
	
	case 42:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_rightline() ){
			pattern = 50;
			cnt1 = 0;
		}
		break;
	
	case 50:
		/* C標的 */
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( C ); // C標的
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 51;
		break;
	
	case 51:
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( C ); // C標的
		if( check_leftline() ){
			pattern = 60;
			cnt1 = 0;
			break;
		}
		if( lEncoderTotal - lEncoderLine >= (273L * 2) ){
			lEncoderLine = lEncoderTotal;
			pattern = 52;
			cnt1 = 0;
		}
		break;
	
	case 52:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_leftline() ){
			pattern = 60;
			cnt1 = 0;
		}
		break;
	
	case 60:
		/* D標的 */
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( D ); // D標的
		lEncoderLine = lEncoderTotal;
		cnt1 = 0;
		pattern = 61;
		break;
	
	case 61:
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( D ); // D標的
		if( check_rightline() || check_crossline()){
			pattern = 20;
			cnt1 = 0;
			lEncoderLine = lEncoderTotal;
			break;
		}
		if( lEncoderTotal - lEncoderLine >= (273L * 2) ){
			lEncoderLine = lEncoderTotal;
			pattern = 62;
			cnt1 = 0;
		}
		break;
	
	case 62:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_rightline() || check_leftline()){
			pattern = 20;
			cnt1 = 0;
			lEncoderLine = lEncoderTotal;
		}
		break;
	
	case 70:
		// 平行標的:振るだけ
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( A ); // A標的
	
		if( check_leftline() ){
			heikou = 0;
			pattern = 20;
			cnt1 = 0;
			break;	
		}
	
		if( check_crossline()){
			heikou=0;
			setBeepPatternS( 0x8800 );
			pattern = 20;
			cnt1 = 0;
			break;	
		}
		if( lEncoderTotal - lEncoderLine >= 273L ){
			pattern = 71;
			lEncoderLine = lEncoderTotal;
			cnt1 = 0;	
		}
		break;
		
	case 71:
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( A );
		if( check_crossline() || check_leftline()){
			heikou=0;
			setBeepPatternS( 0x8800 );
			pattern = 20;
			cnt1 = 0;
			break;	
		}
#if ENC
		if( lEncoderTotal - lEncoderLine >= (UP4BIT * 273L )){
			cnt1 = 0;	
			lEncoderLine = lEncoderTotal;
			pattern = 72;
		}
#endif // ENC 
		break;
	
	case 72:
		servoPwmOut( iServoPwm );
		traceMain();
		servoSet( 2 ); // D,E標的
		if( check_crossline() || check_leftline()){
			setBeepPatternS( 0x8800 );
			heikou=0;
			pattern = 20;
			cnt1 = 0;
			break;	
		}

#if ENC
		if( lEncoderTotal - lEncoderLine >= (MIN4BIT * 273L)){
			lEncoderLine = lEncoderTotal;
			heikou++;
			pattern = 73;
			cnt1 = 0;		
		}
#endif // ENC
		break;
	
	case 73:
		servoPwmOut( iServoPwm );
		traceMain();
		if( check_leftline() ){
			heikou = 0;
			pattern = 20;
			cnt1 = 0;
			break;	
		}
		if( check_crossline() ){
			heikou=0;
			setBeepPatternS( 0x8800 );
			pattern = 20;
			cnt1 = 0;
			break;	
		}
		if( heikou == 1 ){
			pattern = 70;
			cnt1 = 0;
			break;
		}
		if( heikou >= 2 ){
			if( (lEncoderTotal - lEncoderLine >= (273L * 1)) && check_rightline() ){
				lEncoderLine = lEncoderTotal;
				heikou = 0;
				pattern = 20;
				cnt1 = 0;
				break;
			} 
			if( check_leftline()){
				lEncoderLine = lEncoderTotal;
				heikou = 0;
				pattern = 20;
				cnt1 = 0;
				break;
			}
		}
		break;
		
	case 101:
	  	servoPwmOut( iServoPwm );
		led_out( 0x00 );
		motor_mode_f( FREE, FREE );
		motor_mode_r( FREE, FREE );
		motor2_f( 0, 0 );
       	motor2_r( 0, 0 );
		msdFlag = 0;
		if( microSDProcessEnd() == 0 && iEncoder <= 0){
			cnt1 = 0;
			pattern = 102;	
			setBeepPatternS( 0xcc00 );
		}
		break;
	
	case 102:
	  	servoPwmOut( 0 );
		motor_mode_f( BRAKE, BRAKE );
		motor_mode_r( BRAKE, BRAKE );
		motor2_f( 0, 0 );
       	motor2_r( 0, 0 );
		for( i = 1;i < 8; i++ ){
			if (i % 2 == 0){
				led_out( 0xff );	
			} 
			else{
				led_out( 0x00 );	
			}
			fullColor_out( i );
			timer_ms( 200 );
		}
		break;
	
	case 200:
	   /* 円柱標的に突っ込む */
	   	servoSet( 2 );
		
		// iSetAngle = ENCHU_ANGLE; // 微妙に左曲げ
	   	iSetAngleAD = ENCHU_ANGLE_AD;  // 微妙に左曲げ
		servoPwmOut( iServoPwm2 );
		
	   	motor_mode_f( BRAKE, BRAKE );
		motor_mode_r( BRAKE, BRAKE );
		motor2_f( diff(30), 30 );
       	motor2_r( diff(30), 30 );
		
		setBeepPatternS( 0xffff );
		fullColor_out( 5 );
		led_out( 0xc3 );
#if ENC
		if( lEncoderTotal - lEncoderLine >= ( 272L * 20 ) ){  // 本番は 272L * 20
			pattern = 101;	
			cnt1 = 0;
		}
#else
		if( cnt1 > 1000 ){
			pattern = 101;	
			cnt1 = 0;
		}
#endif
		break;
	
	case 999:
		servoPwmOut( 0 );
		motor_mode_f( BRAKE, BRAKE );
		motor_mode_r( BRAKE, BRAKE );
		motor2_f( 0, 0 );
       	motor2_r( 0, 0 );
		break;

    default:
		msdFlag = 0;
		led_out( 0xaa );
	  	servoPwmOut(0);
	  	motor_mode_f( FREE, FREE );
        motor_mode_r( FREE, FREE );
		motor_f( diff(0), 0 );
		motor_r( diff(0), 0 );
        break;
    }
  }
}


/************************************************************************/
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void )
{
  int i;

  init_xin_clk();  // レジスタの初期化

  /* クロックをXINクロック(20MHz)に変更 */
  prc0  = 1;                          /* プロテクト解除               */
  cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
  cm05  = 0;                          /* XINクロック発振              */
  for (i = 0; i < 50; i++ );          /* 安定するまで少し待つ(約10ms) */
  ocd2  = 0;                          /* システムクロックをXINにする  */
  prc0  = 0;

  /* ポートの入出力設定 */

  /*  PWM(予備)       左前M_PMW       右前M_PWM       ブザー
      センサ左端      センサ左中      センサ右中      センサ右端  */
  p0   = 0x00;
  prc2 = 1;                           /* PD0のプロテクト解除          */
  pd0  = 0xf0;

  /*  センサ中心      ｽﾀｰﾄﾊﾞｰ         RxD0            TxD0
      DIPSW3          DIPSW2          DIPSW1          DIPSW0         */
  pur0 |= 0x04;                       /* P1_3?P1_0のプルアップON     */
  p1  = 0x00;
  pd1 = 0x10;

  /*  右前M_方向      ステアM_方向    ステアM_PWM     右後M_PWM
      右後M_方向      左後M_PWM       左後M_方向      左前M_方向      */
  p2  = 0x00;
  pd2 = 0xff;

  /* !---追加・変更---! */
  /*  Arduino(ANGLE)  none            none            none
      none            none            none            エンコーダA相   */
  p3  = 0x00;
  pd3 = 0xfe;

  /*  XOUT            XIN             ボード上のLED   none
      none            VREF            none            none            */
  p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
  pd4 = 0xb8;

  /*  none            none            none            none
      none            none            none            none            */
  p5  = 0x00;
  pd5 = 0xff;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xff;

  /*  DCモータ回転方向1   DCモータ回転方向2       CN6.4入力       CN6.5入力
      none(ｱﾅﾛｸﾞ予備) 角度VR          センサ_左ｱﾅﾛｸﾞ  センサ_右ｱﾅﾛｸﾞ  */
  p7  = 0x00;
  pd7 = 0xc0;

  /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
      DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
  pur2 |= 0x03;                       /* P8_7?P8_0のプルアップON      */
  p8  = 0x00;
  pd8 = 0x00;

  /*  -               -               ﾌﾟｯｼｭｽｲｯﾁ       P8制御(LEDorSW)
      右前M_Free      左前M_Free      右後M_Free      左後M_Free      */
  p9  = 0x00;
  pd9 = 0x1f;
  pu23 = 1;   // P9_4,P9_5をプルアップする

  /* タイマRBの設定 */
  /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                  = 1 / (20*10^6) * 200        * 100
                  = 0.001[s] = 1[ms]
  */
  trbmr  = 0x00;                      /* 動作モード、分周比設定       */
  trbpre = 200 - 1;                   /* プリスケーラレジスタ         */
  trbpr  = 100 - 1;                   /* プライマリレジスタ           */
  trbic  = 0x06;                      /* 割り込み優先レベル設定       */
  trbcr  = 0x01;                      /* カウント開始                 */

  /* A/Dコンバータの設定 */
//  admod   = 0x33;                     /* 繰り返し掃引モードに設定     */
//  adinsel = 0x90;                     /* 入力端子P7の4端子を選択      */
//  adcon1  = 0x30;                     /* A/D動作可能                  */
//  _asm(" NOP ");                      /* φADの1サイクルウエイト入れる*/
//  adcon0  = 0x01;                     /* A/D変換スタート              */

  /* タイマRG タイマモード(両エッジでカウント)の設定 */
  timsr = 0x40;                       /* TRGCLKA端子 P3_0に割り当てる */
  trgcr = 0x15;                       /* TRGCLKA端子の両エッジでカウント*/
  trgmr = 0x80;                       /* TRGのカウント開始            */

  /* タイマRC PWMモード設定(左前モータ、右前モータ) */
  trcpsr0 = 0x40;                     /* TRCIOA,B端子の設定           */
  trcpsr1 = 0x33;                     /* TRCIOC,D端子の設定           */
  trcmr   = 0x0f;                     /* PWMモード選択ビット設定      */
  trccr1  = 0x8e;                     /* ｿｰｽｶｳﾝﾄ:f1,初期出力の設定    */
  trccr2  = 0x00;                     /* 出力レベルの設定             */
  trcgra  = TRC_MOTOR_CYCLE - 1;      /* 周期設定                     */
  trcgrb  = trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
  trcgrc  = trcgrc_buff = trcgra;     /* P0_7端子のON幅(予備)         */
  trcgrd  = trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
  trcic   = 0x07;                     /* 割り込み優先レベル設定       */
  trcier  = 0x01;                     /* IMIAを許可                   */
  trcoer  = 0x01;                     /* 出力端子の選択               */
  trcmr  |= 0x80;                     /* TRCカウント開始              */

  /* タイマRD リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ、ｻｰﾎﾞﾓｰﾀ) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x20;                     /* ソースカウントの選択:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* 周期設定             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅(左後モータ)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅(右後モータ)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5端子のON幅(サーボモータ) */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
  unsigned char b;
  unsigned int i;
  float v;
  static int line_no; /* 行番号 */
  static int cnt2=0;

  _asm(" FSET I ");   /* タイマRB以上の割り込み許可   */

  cnt1++;
  cnt_run++;
  check_sen_cnt++;
  check_enc_cnt++;
  check_cross_cnt++;


  /* サーボモータ制御 */
  servoControl();
  servoControl2();

  /* ブザー処理 */
  beepProcessS();
  
  /* microSD間欠書き込み処理(1msごとに実行) */
  microSDProcess();
	

  if( pattern >= 1 && pattern < 102){
	 servoPwmOut(iServoPwm);
  }
  b = p3_0;
  // エンコーダの信号をLEDに表示
	
  if ( cnt2 >= 600 ) cnt2 = 0;
  if ( cnt2 < 300 ) {
  	led_out( 0x80 | b );
  }
  else if ( cnt2 < 600 ) {
    led_out( 0x40 | b );
  }


  /* 10回中1回実行する処理 */
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* エンコーダ制御 */
      i = trg;
      iEncoder       = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      if ( iEncoder > iEncoderMax ) {
        iEncoderMax = iEncoder;
      }
      uEncoderBuff   = i;
      break;

    case 2:
      /* スイッチ読み込み準備 */
      p9_4 = 0;                       /* LED出力OFF                   */
      pd8  = 0x00;
      break;

    case 3:
      /* スイッチ読み込み、LED出力 */
      types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
      p8  = types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のLEDへ出力*/
      pd8 = 0xff;
      p9_4 = 1;                       /* LED出力ON                    */
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;

    case 9: 
	  /* microSD記録処理 */
	  if( msdFlag == 1 ){
	  		v = ((iEncoder*100) / 2725);
			msdPrintf("%4d,%3d,%d,%d,%d,%5d,%4d,%2d,%f,%d\r\n",
				line_no,	// 行番号
				pattern,	// 動作パターン
				check_leftline(),  // デジタル左
				check_rightline(), // デジタル右
				check_crossline(), // クロスラインチェック
				getAnalogSensor(), // アナログセンサ
				getServoAngle(),   // VR(ステアリング角度)
				iEncoder,           // エンコーダ
				v, // 速度[m/s]
				kyoritime // kyoritime
				
			);
			if(++line_no >= 10000 ) line_no = 0; 
	  }
      break;

    case 10:
      /* iTimer10変数の処理 */
      iTimer10 = 0;
      break;
  }
}

/************************************************************************/
/* タイマRC 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;  /* フラグクリア */

  /* タイマRC　デューティ比の設定 */
  trcgrb = trcgrb_buff;
  trcgrc = trcgrc_buff;
  trcgrd = trcgrd_buff;
}

/************************************************************************/
/* カーブ検出処理(進入時のみ)                                           */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
/************************************************************************/
int check_crossline( void )
{
  int ret = 0;
  int angle;

  angle = getServoAngle();

  //if ( abs( angle ) <= 2 ) { // ハンドルがほぼ直線のとき
    if ( check_leftline() && check_rightline()) { // 左右のデジタルセンサが反応していればカーブ認識
      ret = 1;
    }
  //}

  return ret;
}

/************************************************************************/
/* 右ハーフライン検出処理                        */
/* 引数 なし                              */
/* 戻り値 0:ハーフラインなし 1:あり                  */
/************************************************************************/
int check_rightline( void )
{
  int ret = 0;
  int r = 1-RIGHT;

  if ( r == 1) {
    ret = 1;
  }
  return ret;
}

/************************************************************************/
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_leftline( void )
{
  int ret = 0;
  int l = 1-LEFT; 
  if ( l == 1) {
    ret = 1;
  }
  return ret;
}


/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned char sensor_inp(void)
{
	unsigned char sensor;
	sensor = (~p0_0*2 + ~p1_6) & 0x0f; 
  	return sensor;
}

unsigned char center_inp( void )
{
	unsigned char c;
	c = ~p0_3 & 0x01;
	return c;
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
  unsigned char sw;

  sw = p1 & 0x0f;                     /* P1_3?P1_0読み込み           */

  return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
  /* 実際の入力はタイマRB割り込み処理で実施 */
  return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
  unsigned char sw;

  sw = ~p9_5 & 0x01;

  return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
  /* 実際の出力はタイマRB割り込み処理で実施 */
  types_led = led;
}

/************************************************************************/
/* フルカラーLEDの制御                                          */
/* 引数　 点灯パターン                                             */
/* 戻り値 なし                                                          */
/************************************************************************/
void fullColor_out( unsigned char type )
{
  //p6 = ~fullColor_data[ type ];
}


/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	
	accele_l = -accele_l;
	
    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
	

    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* 左後モータ */
	
	
	accele_l = -accele_l;
	
    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
	

    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
  int sw_data;

  sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
  accele_l = accele_l * sw_data / 20;
  accele_r = accele_r * sw_data / 20;
  
  accele_l = -accele_l;


  /* 左前モータ */
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }

  /* 右前モータ */
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
  accele_l = -accele_l;


  /* 左前モータ */
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }

  /* 右前モータ */
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}


/************************************************************************/
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_0 = 1;
  } else {
    p9_0 = 0;
  }
  if ( mode_r ) {
    p9_1 = 1;
  } else {
    p9_1 = 0;
  }
}

/************************************************************************/
/* 前モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_2 = 1;
  } else {
    p9_2 = 0;
  }
  if ( mode_r ) {
    p9_3 = 1;
  } else {
    p9_3 = 0;
  }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
  if ( pwm >= 0 ) {
    p2_6 = 0;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
  } else {
    p2_6 = 1;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -pwm ) / 100;
  }

}

//************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( get_ad(14) - iAngle0 );
}

/************************************************************************/
/* アナログセンサ値取得                                                 */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
  int ret;

  //  = 左  -  右
  ret = (get_ad(12)) - get_ad(13);                    /* アナログセンサ情報取得       */

  return ret;
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl( void )
{
  int i, iRet, iP, iD;

  i = getAnalogSensor();
  
  /* サーボモータ用PWM値計算 */
  iP = KP * i;                        /* 比例                         */
  iD = KD * ( iSensorBefore - i );    /* 微分(目安はPの5?10倍)       */
  iRet = iP - iD;
  iRet /= 64;

  /* PWMの上限の設定 */
  if ( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX; /* マイコンカーが安定したら     */
  if ( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX; /* 上限を90くらいにしてください */
  iServoPwm = iRet;

  iSensorBefore = i;                  /* 次回はこの値が1ms前の値となる*/
}


/************************************************************************/
/* サーボモータ制御 角度指定用											*/
/* 引数 なし 															*/
/* 戻り値 グローバル変数 iServoPwm2 に代入 								*/
/************************************************************************/
void servoControl2( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!追加・変更!!! */
	// i = iSetAngle; 						/* 設定したい角度 	*/
	// j = getServoAngle(); 				/* 現在の角度 		*/
	
	
	i = iSetAngleAD; 						/* 設定したい角度 	*/
	j = get_ad(14);				 			/* 現在の角度 		*/
	
	
	/* サーボモータ用PWM値計算 */
	iP = 5 * (j - i); 					/* 比例 */
	iD = 25 * (iAngleBefore2 - j); 		/* 微分 */
	
	iRet = iP - iD;
	iRet /= 2;
	iRet = -iRet;
	
	if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;	/* マイコンカーが安定したら 	*/
	if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX; 	/* 上限を90くらいにしてください */
	
	iServoPwm2 = iRet;
	iAngleBefore2 = j;
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
  int i, ret;

  i  = getServoAngle() / AD_1DEG;           /* 1度あたりの増分で割る        */
  if ( i <  0 ) i = -i;
  if ( i > 45 ) i = 45;
  ret = revolution_difference[i] * pwm / 100;

  return ret;
}

/************************************************************************/
/* 10進数→BCD値                            */
/* 引数    BCDにしたい値(int型)                     */
/* 戻り値  BCD値                            */
/* メモ    BCD符号化範囲は0?99まで                    */
/************************************************************************/
unsigned char convertBCD( int data )
{
  unsigned char b1, b2;
  unsigned char ret;

  b1 = data / 10;
  b2 = data % 10;
  ret = b1 * 0x10 + b2;

  return ret;
}

int isRolling( void ){
	int ret = 0;
	
	if((iAngle0) -  (getServoAngle()) < ROLLING){
		ret = 1;	
	}
	else{
		ret = 0;	
	}
	
	return ret;		
} 

/************************************************************************/
/* ライントレース関数本体                                               */
/* 引数   デューティ比(0?100)                                          */
/* 戻り値 なし                                                          */
/* 備考 機体に合わせてパラメータを書き換えてください                    */
/************************************************************************/
/*
void traceMain( void )
{
  const int _3MS = 86;
  const int _2MS = 43;

  int i;
  i = getServoAngle();

  if ( i > 50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( 40, diff(40) );
      motor2_r( 40, diff(40) );
    }
  } else if ( i < -50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( diff(40), 40 );
      motor2_r( diff(40), 40 );
	} 
  
  }else if ( i > 15 ) {
    if (iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( -50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s以上なら
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor2_f( 45, 0 );  // 内輪を0%にして、カーブを曲がりやすくする
      motor2_r( 45, 0 );
    } else {
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor2_f( 50, diff(50) );
      motor2_r( 50, diff(50) );
    }
  } else if ( i < -15 ) {
    if ( iEncoder >= _3MS ) {
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( -50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s以上なら
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( 0, 45 );         // 内輪を0%にして、カーブを曲がりやすくする
      motor2_r( 0, 45 );
    } else {
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( diff(50), 50 );
      motor2_r( diff(50), 50 );
    }
  } else { 
	
    
	if ( iEncoder >= (dipsw_get() * 2 + 75) ) { // 50(2.0m/s)?138(5.0m/s)  // 安定値:15*2+14 = 44
      // dip_swの値↓
      // 0→0*2+25=25(2.3m/s) 8→ 8*2+25=41(3.7m/s)
      // 1→1*2+25=27(2.5m/s) 9→ 9*2+25=43(3.9m/s)
      // 2→2*2+25=29(2.6m/s) 10→10*2+25=45(4.1m/s)
      // 3→3*2+25=31(2.8m/s) 11→11*2+25=47(4.3m/s)
      // 4→4*2+25=33(3.0m/s) 12→12*2+25=49(4.5m/s)
      // 5→5*2+25=35(3.2m/s) 13→13*2+25=51(4.6m/s)
      // 6→6*2+25=37(3.4m/s) 14→14*2+25=53(4.8m/s)
      // 7→7*2+25=39(3.5m/s) 15→15*2+25=55(5.0m/s)
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( 0, 0 );
      motor2_r( 0, 0 );

    } else {
      motor_mode_f( FREE, FREE );
      motor_mode_r( FREE, FREE );
      motor2_f( 100, 100 );
      motor2_r( 100, 100 );
    }
  }
}
*/



void traceMain( void ){
		int i;
		i = getServoAngle();
        if( i > 170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( diff(60), 60 );
            motor_r( diff(60), 60 );
        } else if( i > 25 ) {
      		motor_mode_f( FREE, BRAKE );
      		motor_mode_r( FREE, BRAKE );
			motor_f( diff(80), 80 );
            motor_r( diff(80), 80 );
        } else if( i < -170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( 60, diff(60) );
            motor_r( 60, diff(60) );
        } else if( i < -25 ) {
			motor_mode_f( BRAKE, FREE );
      		motor_mode_r( BRAKE, FREE );
			motor_f( 80, diff(80) );
            motor_r( 80, diff(80) );
        } else {	
			if( iEncoder <= (dipsw_get() * 2 + 45) ){ //75
      			motor_mode_f( FREE, FREE );
      			motor_mode_r( FREE, FREE );
            	motor2_f( 100, 100 );
            	motor2_r( 100, 100 );
			}
			else{
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            	motor2_f( 0, 0 );
            	motor2_r( 0, 0 );
					
			}
        }	
}


/************************************************************************/
/* 過去の横線の検出回数に応じて標的を見分ける                           */
/* 引数   なし                                                          */
/* 戻り値 変数hyouteki_flagに(平行標的:0 垂直標的:1)が入る              */
/************************************************************************/
void hyouteki_check( void ) { - hitcount;
  switch ( hitcount ) {
    case 0:
      hyouteki_flag = 0;
      break;

    case 1:
      hyouteki_flag = 1;
      break;

    default:
      break;

  }
}

/************************************************************************/
/* 数値をある範囲から別の範囲に変換(Arduinoのmap関数と同等)             */
/*                                                                      */
/* 引数   x: 変換したい数値                                             */
/*        in_min: 現在の範囲の下限                                      */
/*        int_max: 現在の範囲の上限                                     */
/*        out_min: 変換後の範囲の下限                                   */
/*        out_max: 変換後の範囲の上限                                   */
/*                                                                      */
/* 戻り値 変換後の数値 (long)                                           */
/************************************************************************/
long map( long x, long in_min, long in_max, long out_min, long out_max ) {

  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;

}

/************************************************************************/
/* 槍角度取得	                                                        */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getLancerAngle( void )
{
    return( get_ad(15) - iLancer0 );  // TypeS基板AN16(p7_4)のR13を外す
}

/************************************************************************/
/* サーボモータ制御(槍)			                                        */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void lancerPwmOut( int pwm )
{
    if( abs(pwm) < 1 ){
		p7_7 = 1;
		p7_6 = 1;
	}
	else{
	 
		if( pwm >= 0 ) {
			p7_7 = 1;
			p7_6 = 0;
    	} else {
			p7_7 = 0;
			p7_6 = 1;	
        	pwm = -pwm;
    	}
	    if( pwm <= 5 ) {
        	trcgrc = trcgrc_buff = trcgra;
    	} else {
    	    trcgrc_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * pwm / 100;
    	}
	}
}

/************************************************************************/
/* サーボモータ制御 槍       											*/
/* 引数 なし 															*/
/* 戻り値 グローバル変数 iLancerPwm に代入 								*/
/************************************************************************/
void lancerControl( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!追加・変更!!! */
	// i = iSetAngle; 						/* 設定したい角度 	*/
	// j = getServoAngle(); 				/* 現在の角度 		*/
	
	
	i = iSetLancer; 						/* 設定したい角度 	*/
	j = get_ad(15);				 			/* 現在の角度 		*/
	 
	/*     P                            D                      */
  	iRet = (3 * i) - ( 5 * (i - iLancerBefore));
    iRet /= 2;
	
	if( iRet >  LANCER_PWM_MAX ) iRet =  LANCER_PWM_MAX;	/* マイコンカーが安定したら 	*/
	if( iRet < -LANCER_PWM_MAX ) iRet = -LANCER_PWM_MAX; 	/* 上限を90くらいにしてください */
	
	iLancerPwm = iRet;
	iLancerBefore = j;
}

void servoSet( int num ) {
  if ( num != 0 ) {
    if ( num == 1 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 2 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 3 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 0;
    }
    if ( num == 4 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 1;
    }
    if ( num == 5 ) {
      SERVO_ANGLE_PORT = 1;
      SERVO_ZERO_PORT  = 0;
      SERVO_MODE_PORT  = 1;
    }
    if ( num == 6 ) {
      SERVO_ANGLE_PORT = 0;
      SERVO_ZERO_PORT  = 1;
      SERVO_MODE_PORT  = 1;
    }
  }
  if ( (num == 0) || (num >= 7) ) {
    SERVO_ANGLE_PORT = 0;
    SERVO_ZERO_PORT  = 0;
    SERVO_MODE_PORT  = 0;
  }
}

void newcrank( void )
{	motor_mode_f(BRAKE,BRAKE);
	motor_mode_r(BRAKE,BRAKE);
				
    if( kyoritime <=   5 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  50 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  75 ){sp(  -50 ,  -50 );}
    else if(cnt1 < 100 ){sp(  -45 ,  -45 );}
    else if(cnt1 < 125 ){sp(  -40 ,  -40 );}
    else if(cnt1 < 150 ){sp(  -30 ,  -30 );}
    else if(cnt1 < 175 ){sp(  -35 ,  -35 );}
    else if(cnt1 < 200 ){sp(  -30 ,  -30 );}
    else if(cnt1 < 225 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 250 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 275 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 300 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 325 ){sp(   diff(60) ,   60 );}
    else if(cnt1 < 350 ){sp(   diff(70) ,   70 );}
    else if(cnt1 < 375 ){sp(   diff(80) ,   80 );}
    else if(cnt1 < 400 ){sp(   diff(90) ,   90 );}
                   else {sp(   diff(100) ,   100 );}
    }
    else if( kyoritime <=   6 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  40 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  65 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  90 ){sp(  -50 ,  -50 );}
    else if(cnt1 < 115 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 140 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 165 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 190 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 215 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 240 ){sp(    0 ,    0 );}
    else if(cnt1 < 265 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 290 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 315 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 340 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 365 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 390 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=   7 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  30 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  55 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  80 ){sp(  -50 ,  -50 );}
    else if(cnt1 < 105 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 130 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 155 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 180 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 205 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 230 ){sp(    0 ,    0 );}
    else if(cnt1 < 255 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 280 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 305 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 330 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 355 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 380 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=   8 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  20 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  45 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  70 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  95 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 120 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 145 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 170 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 195 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 220 ){sp(    0 ,    0 );}
    else if(cnt1 < 245 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 270 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 295 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 320 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 345 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 370 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=   9 ){     /* クロスライン通過後の減速制御 */
         if(cnt1 <  10 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  35 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  60 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  85 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 110 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 135 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 160 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 185 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 210 ){sp(    0 ,    0 );}
    else if(cnt1 < 235 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 260 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 285 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 310 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 335 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 360 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  10 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  50 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  75 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 100 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 125 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 150 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 175 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 200 ){sp(    0 ,    0 );}
    else if(cnt1 < 225 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 250 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 275 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 300 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 325 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 350 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  11 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  40 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  65 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  90 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 115 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 140 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 165 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 190 ){sp(    0 ,    0 );}
    else if(cnt1 < 215 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 240 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 265 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 290 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 315 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 340 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  12 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(  -40 ,  -40 );}
    else if(cnt1 <  30 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  55 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  80 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 105 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 130 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 180 ){sp(    0 ,    0 );}
    else if(cnt1 < 205 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 230 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 255 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 280 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 305 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 330 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  13 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  20 ){sp(  -80 ,  -80 );}
    else if(cnt1 <  45 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  70 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  95 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 120 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 145 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 170 ){sp(    0 ,    0 );}
    else if(cnt1 < 195 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 220 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 245 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 270 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 295 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 320 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  14 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  10 ){sp(  -80 ,  -80 );}
    else if(cnt1 <  35 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  60 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  85 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 110 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 135 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 160 ){sp(    0 ,    0 );}
    else if(cnt1 < 185 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 210 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 235 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 260 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 285 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 310 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  15 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  50 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  75 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 125 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 150 ){sp(    0 ,    0 );}
    else if(cnt1 < 175 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 200 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 225 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 250 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 275 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 300 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  16 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  40 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  65 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  90 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 115 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 140 ){sp(    0 ,    0 );}
    else if(cnt1 < 165 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 190 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 215 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 240 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 265 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 290 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  17 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  30 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  55 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  80 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 105 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 130 ){sp(    0 ,    0 );}
    else if(cnt1 < 155 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 180 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 205 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 230 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 255 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 280 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  18 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  20 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  45 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  70 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  95 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 145 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 170 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 195 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 220 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 245 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 270 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  19 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  10 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  35 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  60 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  85 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 110 ){sp(    0 ,    0 );}
    else if(cnt1 < 135 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 160 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 185 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 210 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 235 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 260 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  20 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  50 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  75 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 100 ){sp(    0 ,    0 );}
    else if(cnt1 < 125 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 150 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 175 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 200 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 225 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 250 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  21 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  40 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  65 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  90 ){sp(    0 ,    0 );}
    else if(cnt1 < 115 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 140 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 165 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 190 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 215 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 240 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  22 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  30 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  55 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  80 ){sp(    0 ,    0 );}
    else if(cnt1 < 105 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 130 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 155 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 180 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 205 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 230 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  23 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  20 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  45 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  70 ){sp(    0 ,    0 );}
    else if(cnt1 <  95 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 120 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 145 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 170 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 195 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 220 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  24 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  10 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  35 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  60 ){sp(    0 ,    0 );}
    else if(cnt1 <  85 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 110 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 135 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 160 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 185 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 210 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  25 ){     /* クロスライン通過後の減速制御 */
	
    if(cnt1 <  25 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  50 ){sp(    0 ,    0 );}
    else if(cnt1 <  75 ){sp(   diff(10) ,   10 );}
    else if(cnt1 < 100 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 125 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 150 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 175 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 200 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  26 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  40 ){sp(    0 ,    0 );}
    else if(cnt1 <  65 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  90 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 115 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 140 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 165 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 190 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  27 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  30 ){sp(    0 ,    0 );}
    else if(cnt1 <  55 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  80 ){sp(   diff(20) ,   20 );}
    else if(cnt1 < 105 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 130 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 155 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 180 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  28 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  20 ){sp(    0 ,    0 );}
    else if(cnt1 <  45 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  70 ){sp(   diff(20) ,   20 );}
    else if(cnt1 <  95 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 120 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 145 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 170 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  29 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  10 ){sp(    0 ,    0 );}
    else if(cnt1 <  35 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  60 ){sp(   diff(20) ,   20 );}
    else if(cnt1 <  85 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 110 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 135 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 160 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  30 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  50 ){sp(   diff(20) ,   20 );}
    else if(cnt1 <  75 ){sp(   diff(30) ,   30 );}
    else if(cnt1 < 100 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 125 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 150 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  31 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  40 ){sp(   diff(20) ,   20 );}
    else if(cnt1 <  65 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  90 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 115 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 140 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  32 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(   diff(10) ,   10 );}
    else if(cnt1 <  30 ){sp(   diff(20) ,   20 );}
    else if(cnt1 <  55 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  80 ){sp(   diff(40) ,   40 );}
    else if(cnt1 < 105 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 130 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  33 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  45 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  70 ){sp(   diff(40) ,   40 );}
    else if(cnt1 <  95 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 120 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  34 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  35 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  60 ){sp(   diff(40) ,   40 );}
    else if(cnt1 <  85 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 110 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  35 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  50 ){sp(   diff(40) ,   40 );}
    else if(cnt1 <  75 ){sp(   diff(50) ,   50 );}
    else if(cnt1 < 100 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  36 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  15 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  40 ){sp(   diff(40) ,   40 );}
    else if(cnt1 <  65 ){sp(   diff(50) ,   50 );}
    else if(cnt1 <  90 ){sp(   diff(60) ,   60 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  37 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <   5 ){sp(   diff(30) ,   30 );}
    else if(cnt1 <  30 ){sp(   diff(40) ,   40 );}
    else if(cnt1 <  55 ){sp(   diff(50) ,   50 );}
    else if(cnt1 <  80 ){sp(   diff(70) ,   70 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  38 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  20 ){sp(   diff(50) ,   50 );}
    else if(cnt1 <  45 ){sp(   diff(60) ,   60 );}
    else if(cnt1 <  70 ){sp(   diff(70) ,   70 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  39 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  10 ){sp(   diff(50) ,   50 );}
    else if(cnt1 <  35 ){sp(   diff(60) ,   60 );}
    else if(cnt1 <  60 ){sp(   diff(70) ,   70 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else if( kyoritime <=  40 ){     /* クロスライン通過後の減速制御 */

    if(cnt1 <  25 ){sp(   diff(60) ,   60 );}
    else if(cnt1 <  50 ){sp(   diff(70) ,   70 );}
                   else {sp(   diff(80) ,   80 );}
    }
    else                       {     /* クロスライン通過後の減速制御 */
    if(cnt1 <  15 ){sp(   diff(60) ,   60 );}
    else if(cnt1 <  40 ){sp(   diff(70) ,   70 );}
                   else {sp(   diff(80) ,   80 );}
    }
}


void sp( int l , int r ){
	servoPwmOut( iServoPwm );
	motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( BRAKE, BRAKE );
	
	motor2_f( l, r );
	if(cnt1 < 100){
		motor2_r( -20, -20 );	
	}
	else{
		motor2_r( 30, diff(30) );
	}
}