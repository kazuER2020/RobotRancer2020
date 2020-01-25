/* ナタデココ用　サーボ処理(R8と連携) */
#include <Servo.h>

#define SERVO_CENTER   70 // サーボの中心位置(各機体に合わせて調整が必要) //hs-311: 78
#define RIGHT_HEIKOU   85 // 槍を右に振る時の角度(平行標的)
#define RIGHT_SUICHOKU_A 50 // (※A標的)槍を右に振る時の角度(垂直標的)
#define LEFT_SUICHOKU_B -50 // 槍を左に振る時の角度(垂直標的)
#define RIGHT_SUICHOKU_C 50 // (※C標的)槍を右に振る時の角度(垂直標的)
#define LEFT_SUICHOKU_D -50 // 槍を左に振る時の角度(垂直標的)

const int DEBUG_LED = 13;
const int SERVO     = 9;           // サーボが接続されるピン
const int SERVO_ANGLE_PORT  = A3;  //14 R8との接続:サーボ信号
const int SERVO_ZERO_PORT   = A4;  //15 R8との接続:角度が0かどうか
const int SERVO_MODE_PORT   = A5;  //16 R8との接続:標的ごとのサーボ角度の切り替え
const int SERVO_A6_PORT     = A6;  //17 右:HIGH…A標的(大きいほう) LOW…C標的(小さいほう)
const int SERVO_A7_PORT     = A7;  //18 左:HIGH…B標的 LOW…D標的(一番小さい)
const int SERVO_HEIKOU_PORT = A2;

unsigned long nowtime, starttime;
int i = 0;

Servo rancer;
void servoWrite( int angle );
unsigned int convertR8toArduino( void );

void setup() {
  pinMode( DEBUG_LED, OUTPUT );
  pinMode( SERVO,     OUTPUT );
  pinMode( SERVO_ANGLE_PORT, INPUT );
  pinMode( SERVO_ZERO_PORT , INPUT );
  pinMode( SERVO_MODE_PORT , INPUT );
  pinMode( SERVO_A6_PORT,    INPUT );
  pinMode( SERVO_A7_PORT,    INPUT );
  pinMode( SERVO_HEIKOU_PORT, INPUT ); // A2は入力設定のみ

  rancer.attach( SERVO );
  servoWrite( 0 );
  starttime = millis();

  /*
    while ( 1 ) {
      servoWrite( 0 );
      delay(1000);
      servoWrite( RIGHT_HEIKOU );
      delay(1000);
      servoWrite( LEFT_SUICHOKU_D );
      delay(1000);
    }
  */
}

void loop() {
  int data = convertR8toArduino();
  nowtime = millis();

  if ( nowtime - starttime > 50 ) {
    starttime = nowtime;
    i = 1 - i;
    digitalWrite(LED_BUILTIN, i );
  }

  switch ( data ) {
    case 0:
      /* 通常状態 */
      servoWrite( 0 );
      break;

    case 1:
      /* 平行振るだけ */
      servoWrite( 35 );
      break;

    case 2:
      /* 平行当てる */
      servoWrite( RIGHT_HEIKOU );
      break;

    case 3:
      /* 垂直A */
      servoWrite( RIGHT_SUICHOKU_A );
      break;

    case 4:
      /* 垂直B */
      servoWrite( LEFT_SUICHOKU_B );
      break;

    case 5:
      /* 垂直C */
      servoWrite( RIGHT_SUICHOKU_C );
      break;

    case 6:
      /* 垂直D */
      servoWrite( LEFT_SUICHOKU_D );
      break;

    default:
      /* NOT REACHED */
      break;
  }
}

/************************************************************************/
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90～90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void servoWrite( int angle ) {
  rancer.write( SERVO_CENTER + angle );  // サーボが左右逆に動く場合は、「-」を「+」に替えてください
}

unsigned int convertR8toArduino( void ) {
  unsigned int a, z, m, a6, a7;
  unsigned int ret;

  a = digitalRead( SERVO_ANGLE_PORT );
  z = digitalRead( SERVO_ZERO_PORT );
  m = digitalRead( SERVO_MODE_PORT );
  a6 = digitalRead( SERVO_A6_PORT );
  a7 = digitalRead( SERVO_A7_PORT );

  ret = (4 * m) + (2 * z) + a;

  return ret;
}

