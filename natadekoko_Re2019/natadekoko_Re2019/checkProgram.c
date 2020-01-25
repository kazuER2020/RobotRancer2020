/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     ���[�^�h���C�u���TypeS Ver.4 or 4.1�E                      */
/*                           �A�i���O�Z���T���TypeS Ver.2 �e�X�g�v���O���� */
/* �o�[�W����   Ver.2.10                                                    */
/* Date         2016.01.25                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/

/*
�{�v���O�����́A
�����[�^�h���C�u���TypeS Ver.3 ����� Ver.4 ����� Ver.4.1   
���A�i���O�Z���T���TypeS Ver.1 ����� Ver.2				  
�̊e��̓���m�F���s���v���O�����ł��B
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "r8c38a_lib.h"
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "printf_lib.h"                 /* printf�g�p���C�u����         */
#include "types3_beep.h"                /* �u�U�[�ǉ�                   */

/*======================================*/
/* �V���{����`                         */
/*======================================*/
/* �萔�ݒ� */
#define     TRC_MOTOR_CYCLE     20000   /* ���O,�E�O���[�^PWM�̎���     */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* ����,�E��,����Ӱ�PWM�̎���   */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     FREE                1       /* ���[�^���[�h�@�t���[         */
#define     BRAKE               0       /* ���[�^���[�h�@�u���[�L       */


#define		LANCER_PWM_MAX		100		/* ���̍ő�PWM  		*/

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
void init_trg_1sou( void );
void init_trg_2sou( void );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char startbar_get( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char cn6_get( void );
void led_out( unsigned char led );
void motor_r( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
void beep_out( int flag );
void lancerControl( void );
void lancerPwmOut( int pwm );


/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
int             pattern;                /* �}�C�R���J�[����p�^�[��     */
unsigned long   cnt1;                   /* �^�C�}�p                     */

/* �G���R�[�_�֘A */
int             iTimer10;               /* 10ms�J�E���g�p               */
long            lEncoderTotal;          /* �ώZ�l�ۑ��p                 */
int             iEncoder;               /* 10ms���̍ŐV�l               */
unsigned int    uEncoderBuff;           /* �v�Z�p�@���荞�ݓ��Ŏg�p     */

/* TRC���W�X�^�̃o�b�t�@ */
unsigned int    trcgrb_buff;            /* TRCGRB�̃o�b�t�@             */
unsigned int    trcgrc_buff;
unsigned int    trcgrd_buff;            /* TRCGRD�̃o�b�t�@             */

/* ���[�^�h���C�u���TypeS Ver.4���LED�A�f�B�b�v�X�C�b�`���� */
unsigned char   types_led;              /* LED�l�ݒ�                    */
unsigned char   types_dipsw;            /* �f�B�b�v�X�C�b�`�l�ۑ�       */
/* ��(DC���[�^�ƃ{�����[��ad7) */
volatile int             iLancer0;				 /* ���S����A/D�l�ۑ� 			 */
volatile int			 iSetLancer;			 /* �ڕW��getLancerAngle()�̒l   */
volatile int 			 iSetLancerAD;			 /* �ڕW��AD�l					 */
volatile int			 iLancerPwm;			 
volatile int			 iLancerBefore;
volatile int             integral_lancer;
/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    int             i, j;
    unsigned int    u;
    char            c;

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */

    /* �}�C�R���J�[�̏�ԏ����� */
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );
    motor_f( 0, 0 );
    motor_r( 0, 0 );
    servoPwmOut( 0 );
	setBeepPatternS(0xaa00);

    while( 1 ) {
        switch( pattern ) {
        case 0:
            /* ���j���[ */
            printf( "\n\n" );
            printf( "Motor Drive PCB TypeS Ver.4 or 4.1 "
                                "Test Program(R8C/38A Ver.) Ver1.00\n" );
            printf( "\n" );
            printf( "1 : LED�̃e�X�g\n" );
            printf( "2 : �X�C�b�`�̃e�X�g\n" );
            printf( "3 : CN6�̓��̓e�X�g\n" );
            printf( "4 : �u�U�[�̃e�X�g\n" );
            printf( "5 : �G���R�[�_�̃e�X�g\n" );
            printf( "6 : �{�����[���̃e�X�g\n" );
            printf( "7 : �A�i���O�Z���T��̃e�X�g\n" );
            printf( "8 : ���[�^�̃e�X�g\n" );
			printf( "9 :  DC���[�^&���pVR�̃e�X�g\n" );
            printf( "\n" );
            printf( "1-9�̐�������͂��Ă������� " );
            pattern = 1;
            break;

        case 1:
            /* �����̓��� */
            if( get_uart0(&c) ) {
                if( c >= '1' && c <= '9' ) {
                    printf( "%c\n\n" , c );
                    cnt1 = 0;
                    pattern = (c - 0x30) * 10 + 1;
                }
            }
            break;

        case 11:
            /* LED�̃e�X�g */
            printf( "����LED3�`10�����Ԃɓ_�����ł��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 12;
            break;

        case 12:
            led_out( 1 << (cnt1 / 500) );
            if( cnt1 >= 4000 ) cnt1 = 0;
            if( get_uart0(&c) == 1 ) {
                led_out( 0 );
                pattern = 0;
            }
            break;

        case 21:
            /* �X�C�b�`�̃e�X�g */
            printf( "���݃X�C�b�`�̒l��\�����ł��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 22;
            break;

        case 22:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "�f�B�b�vSW(SW2)�̒l : %02x ", dipsw_get2() );
                printf( "�v�b�V��SW(SW3)�̒l : %1x\r", pushsw_get() );
            }
            break;

        case 31:
            /* CN6�̓��̓e�X�g */
            printf( "CN6�ɓ��͂���Ă���l��\�����ł��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 32;
            break;

        case 32:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "CN6�̓��͒l : %1x\r", cn6_get() );
            }
            break;

        case 41:
            /* �u�U�[�̃e�X�g */
            printf( "�u�U�[���e�X�g���܂��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 42;
            break;

        case 42:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                beep_out( 0 );
                break;
            }
            if( cnt1 <= 1000 ) {
                beep_out( 1 );
            } else if( cnt1 <= 2000 ) {
                beep_out( 0 );
            } else {
                cnt1 = 0;
            }
            break;

        case 51:
            /* �G���R�[�_�̃e�X�g */
            printf( "�G���R�[�_�̓��̓p���X����\�����܂��B\n" );
            printf( "1 : 1��(P3_0�[�q�Ƀp���X�����)�̃G���R�[�_�e�X�g\n" );
            printf( "2 : 2��(P3_0��P3_2�[�q�Ƀp���X�����)�̃G���R�[�_�e�X�g\n" );
            printf( "0 : �I��\n" );
            printf( "\n" );
            printf( "0-2�̐�������͂��Ă������� " );
            pattern = 52;
            lEncoderTotal = 0;
            break;

        case 52:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    pattern = 0;
                    break;
                } else if( c >= '1' && c <= '2' ) {
                    printf( "%c\n\n" , c );
                    cnt1 = 0;
                    pattern = (c - 0x31) + 53;
                }
            }
            break;

        case 53:
            printf( "\n" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            init_trg_1sou();
            pattern = 55;
            break;

        case 54:
            printf( "\n" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            init_trg_2sou();
            pattern = 55;
            break;

        case 55:
            led_out( p3_2 * 2 + p3_0 );     // �G���R�[�_�̐M����LED�ɕ\��

            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "�G���R�[�_�̃p���X�l : %8ld\r", lEncoderTotal );
            }
            break;

        case 61:
            /* �{�����[���̃e�X�g */
            printf( "�{�����[���d����\�����ł��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 62;
            break;

        case 62:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                i =  get_ad(14);
                j = (long)500 * i / 1023;
                printf( "�{�����[���̓d�� : %4d (%01d.%02dV)\r",
                                                i, j/100, j%100 );
            }
            break;

        case 71:
            /* �A�i���O�Z���T���TypeS Ver.2�̃e�X�g */
            printf( "�A�i���O�Z���T��̒l��\�����ł��B" );
            printf( "�I�������ǂꂩ�L�[�������Ă��������B\n" );
            pattern = 72;
            break;

        case 72:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
				//printf( "sensor= %1d\r", get_ad( 12 )); 
                printf( "Left=%4d , "
                        "CR=%1d , CL=%1d , Right=%4d            \r",
                         get_ad(7), get_ad(12), get_ad(13), get_ad(4));
							/* �� �@�������@�E�����@�E */
            }
            break;

        case 81:
            /* ���[�^(5��)�̃e�X�g */
            printf( "���[�^�̃e�X�g�����܂��B\n" );
            printf( "1 : ����냂�[�^\n" );
            printf( "2 : �E��냂�[�^\n" );
            printf( "3 : �T�[�{���[�^\n" );
            printf( "4 : ���O���[�^\n" );
            printf( "5 : �E�O���[�^\n" );
            printf( "0 : �I��\n" );
            printf( "\n" );
            printf( "0-5�̐�������͂��Ă������� " );
            pattern = 82;
            break;

        case 82:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    pattern = 0;
                    break;
                } else if( c >= '1' && c <= '5' ) {
                    printf( "%c\n\n" , c );
                    cnt1 = 0;
                    pattern = (c - 0x31) * 10 + 101;
                }
            }
            break;
		
		case 91:
			/* �����[�^�̃e�X�g */
			printf( "��DC���[�^&VR���e�X�g���܂��B\n" );
            printf( "0�L�[�ŏI���ł��B\n" );
			printf( "DC: 1\n" );
            printf( "VR: 2\n" );
			while( !get_uart0(&c));
		 	
            if( c == '0' ) {
				pattern = 0;
				break;	
			}
			if( c == '1' ) {
				cnt1 = 0;
            	i = 0;
            	j = 1;
				pattern = 92;
             	break;
            } else if( c == '2' ) {
                pattern = 93;
				printf( "0�L�[�ŏI���ł��B\n" );
			}
			break;
		
		case 92:
			lancerControl();  // ��	
			if( get_uart0(&c) ) {
                if( c == '0' ) {
                    lancerPwmOut(0);
                    pattern = 0;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }	
   			
			if( i == 0 ){
				printf("��~��    \r");	
				lancerPwmOut(0);
			}
			else if( i == 1 ){
				printf("���]50%%  \r");
				lancerPwmOut(50);	
			}
			else if( i == 2 ){
				printf("�t�]50%%  \r");
				lancerPwmOut(-50);	
			}
			else if( i == 3 ){
				printf("���]100%%  \r");
				lancerPwmOut(100);	
			}
			else if( i == 4 ){
				printf("�t�]100%%  \r");
				lancerPwmOut(-100);	
			}
			else{
				i = 0;	
			}
			break;
		
		case 93:
			printf("��VR= %4d\r", get_ad(16));  //p7_4��ǂ�
			if( get_uart0(&c) ) {
                if( c == '0' ) {
            		pattern = 91;
					break;
				}
			}
			break;

        case 101:
            /* ����냂�[�^�̃e�X�g */
            printf( "����냂�[�^���e�X�g���܂��B\n" );
            printf( "�X�y�[�X�L�[�������ē����ς��܂��B" );
            printf( "0�L�[�ŏI���ł��B\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 102;
            break;

        case 102:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "��~��                     \r" );
                    j = i;
                }
                motor2_r( 0, 0 );
                motor_mode_r( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "���]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 50, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "�t�]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( -50, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "���]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 50, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "�t�]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( -50, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 111:
            /* �E��냂�[�^�̃e�X�g */
            printf( "�E��냂�[�^���e�X�g���܂��B\n" );
            printf( "�X�y�[�X�L�[�������ē����ς��܂��B" );
            printf( "0�L�[�ŏI���ł��B\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 112;
            break;

        case 112:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "��~��                     \r" );
                    j = i;
                }
                motor2_r( 0, 0 );
                motor_mode_r( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "���]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, 50 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "�t�]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, -50 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "���]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, 50 );
                    motor_mode_r( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "�t�]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, -50 );
                    motor_mode_r( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 121:
            /* �T�[�{���[�^�̃e�X�g */
            printf( "�T�[�{���[�^���e�X�g���܂��B\n" );
            printf( "�X�y�[�X�L�[�������ē����ς��܂��B" );
            printf( "0�L�[�ŏI���ł��B\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 122;
            break;

        case 122:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    servoPwmOut( 0 );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 3 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "��~��                     \r" );
                    j = i;
                }
                servoPwmOut( 0 );
                break;
            case 1:
                if( i != j ) {
                    printf( "���]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    servoPwmOut( 50 );
                } else if(  cnt1 <= 2000 ) {
                    servoPwmOut( 0 );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "�t�]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    servoPwmOut( -50 );
                } else if(  cnt1 <= 2000 ) {
                    servoPwmOut( 0 );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 131:
            /* ���O���[�^�̃e�X�g */
            printf( "���O���[�^���e�X�g���܂��B\n" );
            printf( "�X�y�[�X�L�[�������ē����ς��܂��B" );
            printf( "0�L�[�ŏI���ł��B\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 132;
            break;

        case 132:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "��~��                     \r" );
                    j = i;
                }
                motor2_f( 0, 0 );
                motor_mode_f( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "���]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 50, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "�t�]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( -50, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "���]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 50, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "�t�]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( -50, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 141:
            /* �E�O���[�^�̃e�X�g */
            printf( "�E�O���[�^���e�X�g���܂��B\n" );
            printf( "�X�y�[�X�L�[�������ē����ς��܂��B" );
            printf( "0�L�[�ŏI���ł��B\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 142;
            break;

        case 142:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "��~��                     \r" );
                    j = i;
                }
                motor2_f( 0, 0 );
                motor_mode_f( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "���]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, 50 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "�t�]50%% <-> �u���[�L���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, -50 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "���]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, 50 );
                    motor_mode_f( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "�t�]50%% <-> �t���[�@���쒆\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, -50 );
                    motor_mode_f( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        default:
            break;
        }
    }
}

/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void )
{
    int     i;

    /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
    prc0  = 1;                          /* �v���e�N�g����               */
    cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
    cm05  = 0;                          /* XIN�N���b�N���U              */
    for(i=0; i<50; i++ );               /* ���肷��܂ŏ����҂�(��10ms) */
    ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
    prc0  = 0;                          /* �v���e�N�gON                 */

    /* �|�[�g�̓��o�͐ݒ� */

    /*  PWM(�\��)       ���OM_PMW       �E�OM_PWM       �u�U�[
        �Z���T���[      �Z���T����      �Z���T�E��      �Z���T�E�[  */
    p0   = 0x00;
    prc2 = 1;                           /* PD0�̃v���e�N�g����          */
    pd0  = 0xf0;

    /*  �Z���T���S      �����ް         RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0          */
    pur0 |= 0x04;                       /* P1_3�`P1_0�̃v���A�b�vON     */
    p1  = 0x00;
    pd1 = 0x10;

    /*  �E�OM_����      �X�e�AM_����    �X�e�AM_PWM     �E��M_PWM
        �E��M_����      ����M_PWM       ����M_����      ���OM_����      */
    p2  = 0x00;
    pd2 = 0xff;

    /*  none            none            none            none
        none            �G���R�[�_B��   none            �G���R�[�_A��   */
    p3  = 0x00;
    pd3 = 0xfa;

    /*  XOUT            XIN             �{�[�h���LED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;

    /*  none            none            none            none
        none            none            none            none            */
    p5  = 0x00;
    pd5 = 0xff;

    /*  none            none            none            none
        none            none            none            none            */
    p6  = 0x00;
    pd6 = 0xff;

    /*  CN6.2����       CN6.3����       CN6.4����       CN6.5����
        none(��۸ޗ\��) �p�xVR          �Z���T_����۸�  �Z���T_�E��۸�  */
    p7  = 0x00;
    pd7 = 0xc0;

    /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
        DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
    pur2 |= 0x03;                       /* P8_7�`P8_0�̃v���A�b�vON     */
    p8  = 0x00;
    pd8 = 0x00;

    /*  -               -               �߯������       P8����(LEDorSW)
        �E�OM_Free      ���OM_Free      �E��M_Free      ����M_Free      */
    p9  = 0x00;
    pd9 = 0x1f;
    pu23 = 1;   // P9_4,P9_5���v���A�b�v����
    /* �^�C�}RB�̐ݒ� */
    /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
    trbpre = 200-1;                     /* �v���X�P�[�����W�X�^         */
    trbpr  = 100-1;                     /* �v���C�}�����W�X�^           */
    trbic  = 0x07;                      /* ���荞�ݗD�惌�x���ݒ�       */
    trbcr  = 0x01;                      /* �J�E���g�J�n                 */

    /* A/D�R���o�[�^�̐ݒ� */
//    admod   = 0x33;                     /* �J��Ԃ��|�����[�h�ɐݒ�     */
//    adinsel = 0x90;                     /* ���͒[�qP7��4�[�q��I��      */
//    adcon1  = 0x30;                     /* A/D����\                  */
//    asm(" nop ");                       /* ��AD��1�T�C�N���E�G�C�g�����*/
//    adcon0  = 0x01;                     /* A/D�ϊ��X�^�[�g              */

  /* �^�C�}RC PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^) */
  trcpsr0 = 0x40;                     /* TRCIOA,B�[�q�̐ݒ�           */
  trcpsr1 = 0x33;                     /* TRCIOC,D�[�q�̐ݒ�           */
  trcmr   = 0x0f;                     /* PWM���[�h�I���r�b�g�ݒ�      */
  trccr1  = 0x8e;                     /* �������:f1,�����o�͂̐ݒ�    */
  trccr2  = 0x00;                     /* �o�̓��x���̐ݒ�             */
  trcgra  = TRC_MOTOR_CYCLE - 1;      /* �����ݒ�                     */
  trcgrb  = trcgrb_buff = trcgra;     /* P0_5�[�q��ON��(���O���[�^)   */
  trcgrc  = trcgrc_buff = trcgra;     /* P0_7�[�q��ON��(�\��)         */
  trcgrd  = trcgrd_buff = trcgra;     /* P0_6�[�q��ON��(�E�O���[�^)   */
  trcic   = 0x07;                     /* ���荞�ݗD�惌�x���ݒ�       */
  trcier  = 0x01;                     /* IMIA������                   */
  trcoer  = 0x01;                     /* �o�͒[�q�̑I��               */
  trcmr  |= 0x80;                     /* TRC�J�E���g�J�n              */

    /* �^�C�}RD ���Z�b�g����PWM���[�h�ݒ�(����Ӱ��A�E��Ӱ��A�ñӰ�) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0  = 0x20;                     /* �\�[�X�J�E���g�̑I��:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* �����ݒ�             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON��(���ヂ�[�^)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON��(�E�ヂ�[�^)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5�[�q��ON��(�T�[�{���[�^) */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ�                      */
/************************************************************************/
void init_trg_1sou( void )
{
    tstart_trgmr = 0;                   /* TRG�̃J�E���g��~            */
    timsr = 0x40;                       /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
    trgcr = 0x15;                       /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
    trgmr = 0x80;                       /* TRG�̃J�E���g�J�n            */
}

/************************************************************************/
/* �^�C�}RG �ʑ��v�����[�h�̐ݒ�                                        */
/************************************************************************/
void init_trg_2sou( void )
{
    tstart_trgmr = 0;                   /* TRG�̃J�E���g��~            */
    timsr = 0xc0;                       /* TRGCLKA�[�q��P3_0,           */
                                        /*   TRGCLKB�[�q��P3_2�Ɋ��蓖��*/
    trgcntc = 0xff;                     /* �ʑ��v��Ӱ�ނ̶��ĕ��@�w��   */
    trgmr = 0x82;                       /* TRG�̃J�E���g�J�n            */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
    unsigned int    i;

    cnt1++;

    /* 10��1����s���鏈�� */
    iTimer10++;
    switch( iTimer10 ) {
    case 1:
        /* �G���R�[�_���� */
        i = trg;
        iEncoder       = i - uEncoderBuff;
        lEncoderTotal += iEncoder;
        uEncoderBuff = i;
        break;

    case 2:
        /* �X�C�b�`�ǂݍ��ݏ��� */
        p9_4 = 0;                       /* LED�o��OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* �X�C�b�`�ǂݍ��݁ALED�o�� */
        types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.4��SW�ǂݍ���*/
        p8  = types_led;                /* ��ײ�ފ��TypeS Ver.4��LED�֏o��*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED�o��ON                    */
        break;

    case 10:
        iTimer10 = 0;
        break;
    }
}

/************************************************************************/
/* �^�C�}RC ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
    trcsr &= 0xfe;

    /* �^�C�}RC�@�f���[�e�B��̐ݒ� */
    trcgrb = trcgrb_buff;
	trcgrc = trcgrc_buff;
    trcgrd = trcgrd_buff;
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃f�W�^���Z���T�l�ǂݍ���              */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���[�A�����A�E���A�E�[�̃f�W�^���Z���T 0:�� 1:��              */
/************************************************************************/
unsigned char sensor_inp( void )
{
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̒��S�f�W�^���Z���T�ǂݍ���            */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���S�f�W�^���Z���T 0:�� 1:��                                  */
/************************************************************************/
unsigned char center_inp( void )
{
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃X�^�[�g�o�[���o�Z���T�ǂݍ���        */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�X�^�[�g�o�[�Ȃ� 1:�X�^�[�g�o�[����                         */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3�`P1_0�ǂݍ���           */

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.4��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    return types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.4��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.4��CN6�̏�ԓǂݍ���                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0�`15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.4��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
    /* ���ۂ̏o�͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    types_led = led;
}

/************************************************************************/
/* ��ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* ���ヂ�[�^ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* ��ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* ���ヂ�[�^ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* �O�ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* �O�ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if( mode_r ) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}

/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if( mode_r ) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �T�[�{���[�^PWM�F-100�`100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
    if( pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -pwm ) / 100;
    }
}

/************************************************************************/
/* �u�U�[����(�e�X�g�v���O������)                                       */
/* �����@ 0:�u�U�[OFF 1:�u�U�[ON                                        */
/* �߂�l �Ȃ�                                                          */
/* ����   ���ۂ̃v���O�����ł́Atypes3_beep.c���g�p���Đ��䂵�Ă������� */
/************************************************************************/
void beep_out( int flag )
{
    if( flag ) {
        p0_4 = 1;
    } else {
        p0_4 = 0;
    }
}


void lancerControl( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!�ǉ��E�ύX!!! */
	// i = iSetAngle; 						/* �ݒ肵�����p�x 	*/
	// j = getServoAngle(); 				/* ���݂̊p�x 		*/
	
	
	i = iSetLancer; 						/* �ݒ肵�����p�x 	*/
	j = get_ad(15);				 			/* ���݂̊p�x 		*/
	
	/* �T�[�{���[�^�pPWM�l�v�Z */
	integral_lancer = j + integral_lancer * (1 / 2);
    
	/*     P       I              D                      */
  	iRet = 3 * i + 3 * integral_lancer + 5 * (i - iLancerBefore);
    iRet /= 2;
	
	if( iRet >  LANCER_PWM_MAX ) iRet =  LANCER_PWM_MAX;	/* �}�C�R���J�[�����肵���� 	*/
	if( iRet < -LANCER_PWM_MAX ) iRet = -LANCER_PWM_MAX; 	/* �����90���炢�ɂ��Ă������� */
	
	iLancerPwm = iRet;
	iLancerBefore = j;
}

/************************************************************************/
/* �T�[�{���[�^����(��)			                                        */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
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
/* end of file                                                          */
/************************************************************************/

/*
�����o��

2011.06.01 Ver.1.00 �쐬
2012.02.23 Ver.1.01 ���[�^�h���C�u���TypeS Ver.3��
                    �A�i���O�Z���T���TypeS Ver.2�̃R�����g�ύX
2013.05.10 Ver.2.00 ���[�^�h���C�u���TypeS Ver.4�ɑΉ�
2013.07.13 Ver.2.01 �R�����g���ꕔ�C��
*/
