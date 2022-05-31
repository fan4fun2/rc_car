#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# ラジコン・カーのリモコンをＲＯＳ経由で制御する       RosRemoCon.py Ver.1.0
#                                                   fan4fun2rc 20 Mar. 2022
#------------------------------------------------------------------------------
#　　ラジコン・カーのリモコンから送られてくる＜左上・左下・右上・右下＞の信号を
#　変換してモーター駆動用のドライバに送る制御プログラムをRadioConCar.pyを作成し
#　たが、操作性が悪くＣＵＩでは状況が見えにくく思った結果は得られなかった。
#　　そこで、Tkinterを使用してＧＵＩ化したGuiRemoCon.pyを作成しＰＣのマウスと
#　キーボードからリモコンするようにしたが、ＰＳ４のリモコンには遠く及ばない。
#　　Raspi0でＲＯＳが動作する環境を知り、ＲＯＳ経由でAndroidのTeleopを使用する
#　ことでRaspi0とＰＣの分散処理ができるようになった。
#　・Raspi0側：RosSpCon.py + RotaryEncoder.py + RosRaspiCheckPower.py
#　・PC側    ：RosBrdgGui.py + GuiRcTest.py
#　今回、ラジコン・カーに付属のリモコンを使用してＲＯＳ経由での操作を追加する。
#　・Raspi0側：RosRemoCon.py（本プログラム）
#　・PC側    ：RosBrdgRc.py
#
#【仕様】
#　・リモコンからの指示は次のRasPiポートに入力される
#　　左上：GPIO25、左下：GPIO24、右上：GPIO23、右下：GPIO22
#　・リモコンの各スティックが押されている間3.0Vで離されている間は0.0V
#　・リモコンのSPEEDYが押されると各ポートへの指示はHiとLoの連続信号
#　・リモコンのSLOWLYが押されると各ポートへの指示は5ms周期の60%比率のPWM信号
#　・リモコン右上で前進、右下で後退とする、左上で左折、左下で右折とする
#　・リモコンのSWを押すたびにSPEEDYで50mm/sずつSLOWLYで10mm/sずつ増加させる
#　・緊急停止は左右の両スティックの同時押しとする
#　・RaspiとPCの分担はRaspiにノード[rc_remocon]を作成しSWの変化情報をトピック
#　　/remocon_infoに配信する
#　・PCではノード[brdg_remocon]を作成しRaspiからの/remocon_infoを購読して
#　　/cmd_velを配信する
#
#【関数概要】
#　１．SW割り込み処理関数：switch_on(引数)
#　２．メイン処理関数：main()
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/20:RadioConCar - v1.2.pyから改変してコーディング開始
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import rospy                            # ROSのPythonライブラリ
from std_msgs.msg import UInt8          # /remocon_info用メッセージ
import RPi.GPIO as GPIO
import time
import copy

#------------------------------------------------------------------------------
#　ハードウェアに接続した入出力ポートをBCM番号で定義する
#　＜注：リモコンの入力ポートは連番がとする必要がある＞
#------------------------------------------------------------------------------
IN_L_U = 25     # リモコンの左SWを上側に押したときの入力GPIOポート
IN_L_D = 24     # リモコンの左SWを下側に押したときの入力GPIOポート
IN_R_U = 23     # リモコンの右SWを上側に押したときの入力GPIOポート
IN_R_D = 22     # リモコンの右SWを下側に押したときの入力GPIOポート

#------------------------------------------------------------------------------
#　モジュール内で使用する定数と変数の定義
#------------------------------------------------------------------------------
SW_ON = 1
SW_OFF = 0
SW_PWM = 2

#==============================================================================
# １．SW割り込み処理関数：switch_on(引数)
#   ＜引数：割り込みを発生させたSWのポート番号＞
#------------------------------------------------------------------------------
#　・変化割込みを発生させたポートを読み込み前回との変化を確認する
#　・リモコンがSLOWLY状態のときはPWMで送信されるので
#　　前回からの変化が10ms以内ならSLOWLY状態と判断する
#　・switch_on.psw[n]：前回のSW情報を保持するリストで右SW下/右SW上/右SW下/右SW上の順
#　　※入力するGPIOポートはIN_R_D/IN_R_U/IN_L_D/IN_L_Uの順に連番である必要がある
#　・switch_on.ptm[n]：前回のSW情報を取り込んだ時刻のリストで書くSWと同じ順
#==============================================================================
def switch_on(sw):
    csw = GPIO.input(sw)                # 現在のSW状態を取り込む
    ctm = time.time()                   # 現在の時刻を取り込む
    n = sw - IN_R_D                     # SWの入力ポートは連番である必要がある
    #GPIO.output(OT_DEB, 1-GPIO.input(OT_DEB))
    if switch_on.psw[n] == csw:         # 一つ前の状態と同じなら処理しない
        switch_on.ptm[n] = ctm          # 次の処理のために現在の時刻を保存
        #print("INT0:" + str(ctm) + ", SW:" + str(switch_on.psw))
        return
    if ctm < switch_on.ptm[n] + 0.01:   # 前回から10ms以下ならPWMと判断しSLOWLYをセット
        switch_on.ptm[n] = ctm          # 次の処理のために現在の時刻を保存
        switch_on.psw[n] = SW_PWM
        #print("INT1:" + str(ctm) + ", SW:" + str(switch_on.psw))
        return
    switch_on.psw[n] = csw              # 次の処理のために現在のSW状態を保存
    switch_on.ptm[n] = ctm              # 次の処理のために現在の時刻を保存
    #print("INT2:" + str(ctm) + ", SW:" + str(switch_on.psw))


#==============================================================================
# ２．メイン処理関数：main()
#------------------------------------------------------------------------------
#　・リモコンからのSW入力につないだGPIOポートを入力に設定する
#　・SWポートを立ち下がりと立ち下がりの両エッジ割り込み処理に設定
#　・無限メインループ：終了条件に(^C)をキャッチできるようにする
#　・SW入力がPWM時の処理、PWM時はSWがOFFになったことを時間で検出する
#　・前回SW状態を配信してから変化があればSW状態を配信する
#　・終了条件の(^C)をキャッチする
#　・終了処理
#==============================================================================
def main():
    GPIO.setmode(GPIO.BCM)      # GPIOの指定にBCM番号を使用する
    #
    # リモコンからのSW入力につないだGPIOポートを入力に設定する
    #
    GPIO.setup(IN_L_U, GPIO.IN)
    GPIO.setup(IN_L_D, GPIO.IN)
    GPIO.setup(IN_R_U, GPIO.IN)
    GPIO.setup(IN_R_D, GPIO.IN)
    #
    # SWポートを立ち下がりと立ち下がりの両エッジ割り込み処理に設定し、
    # 処理関数はswitch_on()
    #
    GPIO.add_event_detect(IN_L_U, GPIO.BOTH, callback=switch_on)
    GPIO.add_event_detect(IN_L_D, GPIO.BOTH, callback=switch_on)
    GPIO.add_event_detect(IN_R_U, GPIO.BOTH, callback=switch_on)
    GPIO.add_event_detect(IN_R_D, GPIO.BOTH, callback=switch_on)
    #
    # 関数内の変数定義と初期化
    #
    ctm = time.time()
    msw = [0,0,0,0]                     # 前回のメインループで変化したSW状態を保持するリストを作成
    switch_on.psw = [0,0,0,0]           # 一つ前のSW状態を保持するリストを作成
    switch_on.ptm = [ctm,ctm,ctm,ctm]   # 一つ前の時刻を保持するリストを作成

    #--------------------------------------------------------------------------
    # ここからメインスレッドでROSのコードが実行される
    #--------------------------------------------------------------------------
    rospy.init_node('rc_remocon')
    rospy.loginfo("ROSノード[rc_remocon]を開始しました。")
    #
    # 配信するトピックを作成する
    #
    pub = rospy.Publisher('/remocon_info', UInt8, queue_size=10)
    rate = rospy.Rate(100)              # 100hz(10ms周期で処理を行う)
    u8 = UInt8()                        # SWの状態を８ビットにパックするメモリ
    s1 = True                           # SWの状態に変化があっても１回待つ
    #
    # 無限メインループ：終了条件に(^C)をキャッチできるようにする
    #
    try:
        while not rospy.is_shutdown():
            ctm = time.time()           # 現在の時刻を取り込む
            #
            # SW入力がPWM時の処理、PWM時はSWがOFFになったことを時間で検出する
            #
            for idx in [i for i, x in enumerate(switch_on.psw) if x == SW_PWM]:
                if ctm - switch_on.ptm[idx] > 0.01: 
                    switch_on.psw[idx] = GPIO.input(idx + IN_R_D)
                    #print("Free:" + str(ctm) + ", SW:" + str(msw))
            #
            # 前回SW状態を配信してから変化があればSW状態を配信する
            #
            if msw != switch_on.psw:
                if s1:              # SWの状態に変化があっても１回待つ
                    s1 = False
                else:
                    s1 = True
                    msw = copy.deepcopy(switch_on.psw)
                    u8 = msw[0] + msw[1]*4 + msw[2]*16 + msw[3]*64
                    pub.publish(u8)         # SW状態をパッキングして配信
                    #rospy.loginfo("SW:" + str(msw))
            #
            rate.sleep()
            #rospy.loginfo("Time:" + str(ctm))  # 最速でも50msに一度程度しか回らない
    #
    # 終了条件の(^C)をキャッチする
    #
    except KeyboardInterrupt:
        pass
    rospy.loginfo("ROSノード[rc_remocon]を終了しました。")
    #
    # 終了処理
    #
    GPIO.cleanup()


#------------------------------------------------------------------------------
# このファイルから起動時のみ実行する
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
