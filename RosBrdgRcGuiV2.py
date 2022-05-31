#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# ラジコン・カーのリモコンをＲＯＳ経由で制御するプログラムのＧＵＩバージョン
#                                                   RosBrdgRcGuiV2.py Ver.2.1
#                                                   fan4fun2rc 29 Apr. 2022
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
#　・Raspi0側：RosRemoCon.py
#　・PC側    ：RosBrdgRc.py（本プログラム）
#　V1.0：更に、リモコンの操作状況が速度で確認できるようにPC側にＧＵＩを追加した
#　・PC側    ：RosBrdgRcGui.py（本プログラム）+ GuiBgrdRc.py
#　V2.0：リモコンの操作仕様を変更し、スティックを離した時に速度を保持する仕様から
#　   ストップするように変更する。そこでV2では、スティックを押した時の速度を
#　   プリセット出来るようにＧＵＩも変更する。
#　・PC側    ：RosBrdgRcGuiV2.py（本プログラム）+ GuiBgrdRcV2.py
#
#【仕様】
#　・リモコンからの指示は次のRasPiポートに入力される
#　　左上：GPIO25、左下：GPIO24、右上：GPIO23、右下：GPIO22
#　・リモコンの各スティックが押されている間3.0Vで離されている間は0.0V
#　・リモコンのSPEEDYが押されると各ポートへの指示はHiとLoの連続信号
#　・リモコンのSLOWLYが押されると各ポートへの指示は5ms周期の60%比率のPWM信号
#　・リモコン右上で前進、右下で後退とする、左上で左折、左下で右折とする
#　・リモコンのSWを押している間モーターを駆動しSWを離すことでストップする
#　・モーターの駆動力はプリセット出来るようにしＧＵＩで変更と表示を可能とする
#　・駆動力の増減は500ms以内のダブルクリックで行う
#　・停止中からのSLOWLYモードはプリセット値の増減とする
#　・RaspiとPCの分担はRaspiにノード[rc_remocon]を作成しSWの変化情報をトピック
#　　/remocon_infoに配信する
#　・PCではノード[brdg_remocon]を作成しRaspiからの/remocon_infoを購読して
#　　/cmd_velを配信する
#　・/cmd_velで配信した並進速度と回転速度をＧＵＩに関数コールで表示する
#　・ＧＵＩからも速度変更が出来るようにコールバック関数を設定してＰＣの画面から
#　　速度の変更や停止が出来るようにする
#
#【関数概要】
#　１．SW割り込み処理関数：rcCallback(引数)
#　２．速度やプリセット値が変更されたときコールされる関数：speedCallback(x,z,px,pz)
#　３．500ms遅れで起動される駆動停止関数：stopDrive(sw)
#　４．メイン処理関数：main()
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/23:RosBrdgRc.pyから改変してコーディング開始
#　・V2.0-2022/03/24:RosBrdgRcGui.pyからリモコンの操作仕様変更
#　・V2.1-2022/04/29:GUIが作成される前に234行目が実行されエラーするので1.5秒から3.5秒に修正
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import rospy                            # ROSのPythonライブラリ
from std_msgs.msg import UInt8          # /remocon_info用メッセージ
from geometry_msgs.msg import Twist     # /cmd_vel用メッセージ
import time
import copy
import GuiBrdgRcV2 as gui
import threading

#------------------------------------------------------------------------------
#　モジュール内で使用する定数と変数の定義
#------------------------------------------------------------------------------
SW_ON = 1
SW_OFF = 0
SW_PWM = 2

#==============================================================================
# １．リモコンのSW情報トピックを受信したときにコールバックされる関数：rcCallback(u8)
#   ＜引数 u8：リモコンのSW情報を下記のようにエンコードしている＞
#------------------------------------------------------------------------------
#　・std_msgs/UInt8.msgを使用して（独自型を作成せず）
#　　‐bit0-1：リモコンの右SWを下側に押したときの状態[SW_OFF/SW_ON/SW_PWM]
#　　‐bit2-3：リモコンの右SWを上側に押したときの状態[SW_OFF/SW_ON/SW_PWM]
#　　‐bit4-5：リモコンの左SWを下側に押したときの状態[SW_OFF/SW_ON/SW_PWM]
#　　‐bit6-7：リモコンの左SWを上側に押したときの状態[SW_OFF/SW_ON/SW_PWM]
#------------------------------------------------------------------------------
#　・パッキングされたSW状態を元に戻す
#　・変化したSWの状態に応じて、速度情報やプリセット情報を変更する
#　・計算された速度指示を配信する
#==============================================================================
def rcCallback(u8):
    sw = u8.data        # 配信されたSW状態を取り込む
    ctm = time.time()   # 現在の時刻を取り込む
    #
    # パッキングされたSW状態を元に戻す
    #
    csw = [0,0,0,0]
    csw[0] = sw % 4; sw //= 4
    csw[1] = sw % 4; sw //= 4
    csw[2] = sw % 4; sw //= 4
    csw[3] = sw % 4
    #print("CB1:" + str(ctm) + ", SW:" + str(csw))
    #
    # 変化したSWを特定し変化した時刻を記録する
    #
    if rcCallback.psw == csw:           # 一つ前の状態と同じなら処理しない
        return
    if rcCallback.psw[0] != csw[0]:
        rcCallback.tx.cancel()
        if csw[0] == SW_ON:             # 前回のオフから500ms以内はダブルクリック
            if rcCallback.ptm[0] + 0.5 > ctm:
                rcCallback.preset_x += 0.05
            rcCallback.vel.linear.x = -rcCallback.preset_x
        elif csw[0] == SW_PWM:  rcCallback.preset_x -= 0.05
        elif csw[0] == SW_OFF:
            if rcCallback.psw[0] == SW_ON:  # 駆動中からのSWオフは500msタイマ起動
                rcCallback.ptm[0] = ctm
                #
                # 50ms後に駆動力のオフ処理を起動する
                #
                rcCallback.tx = threading.Timer(0.5, stopDrive, args=(0, ))
                rcCallback.tx.start()
    if rcCallback.psw[1] != csw[1]:
        rcCallback.tx.cancel()
        if csw[1] == SW_ON:             # 前回のオフから500ms以内はダブルクリック
            if rcCallback.ptm[1] + 0.5 > ctm:
                rcCallback.preset_x += 0.05
            rcCallback.vel.linear.x = rcCallback.preset_x
        elif csw[1] == SW_PWM:  rcCallback.preset_x += 0.05
        elif csw[1] == SW_OFF:
            if rcCallback.psw[1] == SW_ON:  # 駆動中からのSWオフは500msタイマ起動
                rcCallback.ptm[1] = ctm
                #
                # 50ms後に駆動力のオフ処理を起動する
                #
                rcCallback.tx = threading.Timer(0.5, stopDrive, args=(1, ))
                rcCallback.tx.start()
    if rcCallback.psw[2] != csw[2]:
        rcCallback.tz.cancel()
        if csw[2] == SW_ON:             # 前回のオフから500ms以内はダブルクリック
            if rcCallback.ptm[2] + 0.5 > ctm:
                rcCallback.preset_z += 0.05
            rcCallback.vel.angular.z = -rcCallback.preset_z
        elif csw[2] == SW_PWM:  rcCallback.preset_z -= 0.05
        elif csw[2] == SW_OFF:
            if rcCallback.psw[2] == SW_ON:  # 駆動中からのSWオフは500msタイマ起動
                rcCallback.ptm[2] = ctm
                #
                # 50ms後に駆動力のオフ処理を起動する
                #
                rcCallback.tz = threading.Timer(0.5, stopDrive, args=(2, ))
                rcCallback.tz.start()
    if rcCallback.psw[3] != csw[3]:
        rcCallback.tz.cancel()
        if csw[3] == SW_ON:             # 前回のオフから500ms以内はダブルクリック
            if rcCallback.ptm[3] + 0.5 > ctm:
                rcCallback.preset_z += 0.05
            rcCallback.vel.angular.z = rcCallback.preset_z
        elif csw[3] == SW_PWM:  rcCallback.preset_z += 0.05
        elif csw[3] == SW_OFF:
            if rcCallback.psw[3] == SW_ON:  # 駆動中からのSWオフは500msタイマ起動
                rcCallback.ptm[3] = ctm
                #
                # 50ms後に駆動力のオフ処理を起動する
                #
                rcCallback.tz = threading.Timer(0.5, stopDrive, args=(3, ))
                rcCallback.tz.start()
    rcCallback.psw = copy.deepcopy(csw)
    #
    # 計算された速度指示を配信する、GUIへは関数でコールを追加
    #
    rcCallback.pub.publish(rcCallback.vel)
    gui.setSpeed(rcCallback.vel.linear.x, rcCallback.vel.angular.z,
                    rcCallback.preset_x, rcCallback.preset_z)
    #print("CB2:" + str(ctm) + ", Vel:" + str(rcCallback.vel))


#==============================================================================
# ２．速度やプリセット値が変更されたときコールされる関数：speedCallback(x,z,px,pz)
#   ＜引数  x：更新する並進速度情報[m/s]＞
#   ＜引数  z：更新する回転速度情報[rad/s]＞
#   ＜引数 px：更新する並進速度のプリセット値[m/s]＞
#   ＜引数 pz：更新する回転速度のプリセット値[rad/s]＞
#------------------------------------------------------------------------------
#　・受信した速度情報をトピック/cmd_velに配信する
#==============================================================================
def speedCallback(x, z, px, pz):
    rcCallback.vel.linear.x = x
    rcCallback.vel.angular.z = z
    rcCallback.preset_x = px
    rcCallback.preset_z = pz
    rcCallback.pub.publish(rcCallback.vel)


#==============================================================================
# ３．500ms遅れで起動される駆動停止関数：stopDrive(sw)
#   ＜引数 sw：発行したSWのID＞
#------------------------------------------------------------------------------
#　・SWがオンからオフになったとき500ms待ってから停止する
#==============================================================================
def stopDrive(sw):
    if sw == 0 or sw == 1:
        rcCallback.vel.linear.x = 0.0
    else:
        rcCallback.vel.angular.z = 0.0
    rcCallback.pub.publish(rcCallback.vel)
    gui.setSpeed(rcCallback.vel.linear.x, rcCallback.vel.angular.z,
                    rcCallback.preset_x, rcCallback.preset_z)
    #print("CB2:" + str(ctm) + ", Vel:" + str(rcCallback.vel))


#==============================================================================
# ４．メイン処理関数：main()
#------------------------------------------------------------------------------
#　・関数内の変数定義と初期化
#　・ROSのノード[brdg_remocon]を作成し実行する
#　・SPEEDY時とSLOWLY時の並進速度[mm/s]と回転速度[rad/1000s]の増分をパラメータで設定する
#　・配信するトピックの作成と、購読するトピックを定義する
#　・無限メインループ：終了条件に(^C)をキャッチできるようにする
#　・終了条件の(^C)をキャッチする
#　・終了処理
#==============================================================================
def main():
    #
    # 関数内の変数定義と初期化
    #
    ctm = time.time()
    rcCallback.psw = [0,0,0,0]          # 一つ前のSW状態を保持するリストを作成
    rcCallback.ptm = [ctm,ctm,ctm,ctm]  # 一つ前の時刻を保持するリストを作成
    rcCallback.vel = Twist()            # 配信する速度オブジェクトを作成
    rcCallback.vel.linear.x = 0.0       # 並進速度を初期化
    rcCallback.vel.angular.z = 0.0      # 回転速度を初期化
    rcCallback.tx = threading.Timer(0.5, stopDrive, args=(0, ))
    rcCallback.tz = threading.Timer(3.5, stopDrive, args=(2, ))     # 3.5秒に変更
    rcCallback.tz.start()               # プリセット値を表示するために
    #--------------------------------------------------------------------------
    # ROSのノード[brdg_remocon]を作成し実行する
    #--------------------------------------------------------------------------
    rospy.init_node('brdg_remocon')
    rospy.loginfo("ROSノード[brdg_remocon]を開始しました。")
    #
    # 並進速度[m/s]と回転速度[rad/s]のプリセット値をパラメータで設定する
    #
    rcCallback.preset_x = rospy.get_param('~preset_x', 0.2)
    rcCallback.preset_z = rospy.get_param('~preset_z', 0.2)
    #
    # 配信するトピックの作成と、購読するトピックを定義する
    #
    rcCallback.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/remocon_info", UInt8, rcCallback, queue_size=10)
    #
    # ＧＵＩのインターフェース関数を設定
    #
    gui.setCallbackSpeed(speedCallback)
    #
    # メインループはＧＵＩで終了まで実行：終了条件に(^C)をキャッチできるようにする
    #
    try:
        gui.guiApp()
    #
    # 終了条件の(^C)をキャッチする
    #
    except KeyboardInterrupt:
        pass
    #
    # 終了処理
    #
    rospy.loginfo("ROSノード[brdg_remocon]を終了しました。")


#------------------------------------------------------------------------------
# このファイルから起動時のみ実行する
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
