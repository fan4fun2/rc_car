#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# RasPiの+5V電源を監視しROSにメッセージする       RosRaspiCheckPower.py Ver.1.0
#                                               fan4fun2rc 7 Feb. 2022
#------------------------------------------------------------------------------
#　 ラジコン・カーのバッテリー電圧が低下したときにラズパイを自動でシャットダウン
#　する制御プログラムとして、RaspiCheckPower.pyを作成したが自動シャットダウンは
#　コメントアウトしてプリント文で常時表示していた。
#　 ラジコンの制御をＲＯＳに移行したので、今回、RosRaspiCheckPower.pyとして
#　バッテリー電圧を/raspi_powerトピックで配信しRosBrdgGuiV2.py経由でGuiRcTestV2.py
#　に表示して、ＧＵＩからのクリックでシャットダウンできるように変更した。
#
#【仕様】
#　・GPIOコネクタから供給される+5Vのバッテリー電源を監視する
#　・ラズパイにADCが無いのでMCP3425をI2Cで接続している
#　・ADCを1秒に1回チェックして10秒間の平均電圧を/raspi_powerに配信する
#　・/raspi_shellに購読し送られてきたコマンドをシェルで実行する
#　・Raspiの入力電圧+5Vを22kΩと10kΩで分圧してMCP3425に入力している
#
#【関数概要】
#　１．ピック/raspi_shellを受信時のコールバック関数：stringCallback(s)
#　２．シグナル割り込み処理関数：handler(signum, frame)
#　３．メインループ
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/15:RaspiCheckPower-test.pyから修正開始
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import rospy                            # ROSのPythonライブラリ
from std_msgs.msg import Float32        # /raspi_power用メッセージ
from std_msgs.msg import String         # /raspi_shell用メッセージ
from smbus2 import SMBus, i2c_msg
import time
import signal
import sys
import os
import struct

#==============================================================================
# １．ピック/raspi_shellを受信時のコールバック関数：stringCallback(s)
#   ＜引数 s：ラズパイのシェルに与えるコマンド＞
#------------------------------------------------------------------------------
#　・受信したメッセージをシェルのコマンドとして実行する
#==============================================================================
def stringCallback(s):
    rospy.loginfo("[%s]:ラズパイのシェルに与えるコマンド(%s)"
                    % (time.ctime(), str(s.data)))
    os.system(str(s.data))

#==============================================================================
# ２．シグナル割り込み処理関数：handler(signum, frame)
#   ＜引数 signum：割り込みを発生させたシグナル番号＞
#   ＜引数  frame：現在のスタックフレーム＞
#------------------------------------------------------------------------------
#　・終了時刻と終了の原因となったシグナル番号を表示する
#　・プログラムを終了させる
#==============================================================================
def handler(signum, frame):
    rospy.loginfo("[%s]:Exit from Check Raspi Power +5V Node by signal(%d)"
                    % (time.ctime(), signum))
    sys.exit(1)

#------------------------------------------------------------------------------
# シグナルハンドラの設定
#------------------------------------------------------------------------------
signal.signal(signal.SIGHUP, handler)   # 1:ハングアップ
signal.signal(signal.SIGINT, handler)   # 2:キーボードからの^C
signal.signal(signal.SIGQUIT, handler)  # 3:キーボードからの^\
#signal.signal(signal.SIGKILL, handler) # 9:強制終了はトラップできない
signal.signal(signal.SIGTERM, handler)  # 15:システムからの終了シグナル

#------------------------------------------------------------------------------
#　モジュール内で使用する定数と変数の定義
#------------------------------------------------------------------------------
i2c = SMBus(1)  # i2cのインスタンスを作成しチャネルをオープン
addr=0x68       # MCP3425のアドレスは0x68で固定
Vref=2.048      # MCP3425の基準電圧で、この値が変換の最大値になる
#config = [0b1_00_0_10_00]  # bit7:Initiate Conv., bit4:One-Shot, bit3-2:16bit, bit1-0:Gain=1
config = [0b10001000]       # 上述は3.6からなので2.7記述に変更
Vsum=48.0       # 過去10回分の電圧を足した値
v=[4.8, 4.8, 4.8, 4.8, 4.8, 4.8, 4.8, 4.8, 4.8, 4.8]    # 10回分の測定値をメモル
i=0             # 10回分のカウンタ
stm = time.time()

#------------------------------------------------------------------------------
# ＲＯＳノードの作成と開始時刻を出力する
#------------------------------------------------------------------------------
rospy.init_node('check_power')
rospy.loginfo("[%s]:Start Check Raspi Power +5V Node[check_power]." % time.ctime())
#
# トピックを受信したときに通知するコールバック関数を設定する
#
rospy.Subscriber('/raspi_shell', String, stringCallback, queue_size=2)
pub = rospy.Publisher('/raspi_power', Float32, queue_size=10)
f32 = Float32()

#==============================================================================
# 1秒ごとに+5V電源の電圧をチェックする無限ループ
#------------------------------------------------------------------------------
#　・MCP3425をワンショットモードに設定して低電力モードを使用する
#　・1秒後にデータを取り込む、1バイト目：MSB、2バイト目：LSB、3バイト目：Config
#==============================================================================
while True:
    msg = i2c_msg.write(addr, config)
    i2c.i2c_rdwr(msg)
    time.sleep(1)
    #
    # 1秒間パワーセーブしてから取得データを読み込む
    #
    msg = i2c_msg.read(addr, 3)     # Read 3 bytes
    i2c.i2c_rdwr(msg)
    #print(msg.len)
    data = list(msg)
    #print("data[0]=%#x, data[1]=%#x, data[2]=%#x" % (data[0], data[1], data[2]))
    #
    # 2バイトの符号付整数をパイソンの整数に変換
    #
    #raw = int.from_bytes(data[0:2], byteorder='big', signed=True)
    raw = struct.unpack('>h', bytearray(data[0:2]))[0]  # 上述は3.2からなので2.7記述に変更
    raw_s = raw * (10.2 + 22) / 10  # 10.2は誤差の実測調整値
    volts = round((Vref * raw_s / 32767), 4)
    #print("raw=%d, raw_s=%d, volts=%5.3f" % (raw, raw_s, volts))
    #
    # 過去10回分のデータの平均を取る
    #
    Vsum = Vsum - v[i] + volts      # 10回前の分を削除して最新を加える
    v[i] = volts
    i += 1
    i %= 10
    #
    # 計測した電圧を/raspi_powerトピックに配信する
    #
    f32.data = Vsum / 10
    pub.publish(f32)
    
